/* -*- c++ -*- */
/*
 * Copyright 2026 liu.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "gfsk_receiver_noise_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <pmt/pmt.h>

namespace gr {
namespace robomaster {

gfsk_receiver_noise::sptr
gfsk_receiver_noise::make(float threshold, int sps, int packet_bytes)
{
    return gnuradio::make_block_sptr<gfsk_receiver_noise_impl>(threshold, sps, packet_bytes);
}

gfsk_receiver_noise_impl::gfsk_receiver_noise_impl(float threshold, int sps, int packet_bytes)
    : gr::block("gfsk_receiver_noise",
                gr::io_signature::make(1, 1, sizeof(float)),
                gr::io_signature::make(1, 1, sizeof(uint8_t)))
{
    d_threshold = threshold;
    d_sps = sps;
    d_packet_bytes = packet_bytes;

    d_packet_bits = d_packet_bytes * 8;
    d_payload_samples = d_packet_bits * d_sps;

    // 步骤 1：生成 64 bit 干扰帧头的过采样相关模板。
    uint64_t access_code = 0x16E8D377151C712D;
    for (int i = 63; i >= 0; --i)
    {
        int symbol = ((access_code >> i) & 1) ? 1 : -1;
        d_symbol_kernel.push_back(symbol);
        for (int j = 0; j < d_sps; ++j)
        {
            d_kernel.push_back((float)symbol);
        }
    }
    if (d_kernel.empty())
    {
        std::cerr << "Error: d_kernel is empty!" << std::endl;
    }
    d_kernel_len = d_kernel.size();

    d_state = SEARCH;
    d_start_idx = 0;

    // 保留帧头长度的历史样本。
    set_history(d_kernel_len);
}

gfsk_receiver_noise_impl::~gfsk_receiver_noise_impl() {}

// 请求一次搜帧和提取所需的输入样本。
void gfsk_receiver_noise_impl::forecast(int noutput_items,
                                        gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = d_payload_samples + d_kernel_len;
}

// 流程：相关搜帧、符号判决、帧头校验、负载输出。
int gfsk_receiver_noise_impl::general_work(int noutput_items,
                                           gr_vector_int &ninput_items,
                                           gr_vector_const_void_star &input_items,
                                           gr_vector_void_star &output_items)
{
    const float *in = (const float *) input_items[0];
    uint8_t *out = (uint8_t *) output_items[0];

    int ninputs = ninput_items[0];
    int consumed = 0;
    int produced = 0;
    const int output_len = 15;

    // 用符号前缀和加速相关计算。
    d_sign_prefix.resize(ninputs + 1);
    d_sign_prefix[0] = 0;
    for (int i = 0; i < ninputs; ++i)
    {
        d_sign_prefix[i + 1] = d_sign_prefix[i] + ((in[i] > 0) ? 1 : -1);
    }

    auto correlation_at = [this](int pos) {
        int score = 0;
        for (size_t bit = 0; bit < d_symbol_kernel.size(); ++bit)
        {
            int begin = pos + (int)bit * d_sps;
            int end = begin + d_sps;
            score += d_symbol_kernel[bit] * (d_sign_prefix[end] - d_sign_prefix[begin]);
        }
        return score;
    };

    while (consumed <= ninputs - d_kernel_len)
    {

        if (d_state == SEARCH)
        {
            int correlation_score = correlation_at(consumed);

            // 步骤 2：越过门限后对齐局部相关峰值。
            if (correlation_score > d_threshold)
            {
                int search_window = std::min((int)(d_sps * 2), ninputs - d_kernel_len - consumed);

                int max_score = correlation_score;
                int best_offset = 0;

                for (int offset = 1; offset < search_window; ++offset)
                {
                    int temp_score = correlation_at(consumed + offset);
                    if (temp_score > max_score)
                    {
                        max_score = temp_score;
                        best_offset = offset;
                    }
                }

                consumed += best_offset;
                d_state = EXTRACT;
            }
            else
            {
                consumed++;
            }
        }

        else if (d_state == EXTRACT)
        {
            if (ninputs - consumed < d_payload_samples)
            {
                break;
            }

            // 取符号中部样本的加权均值。
            auto symbol_avg = [this, in](int bit_start) {
                int sample_begin = d_sps / 4;
                int sample_end = d_sps - sample_begin;
                float center = (d_sps - 1) * 0.5f;
                float half_width = std::max(1.0f, d_sps * 0.25f);
                float sum_voltage = 0.0f;
                float sum_weight = 0.0f;

                for (int k = sample_begin; k < sample_end; ++k)
                {
                    float weight = half_width + 1.0f - std::abs(k - center);
                    weight = std::max(weight, 1.0f);
                    sum_voltage += in[bit_start + k] * weight;
                    sum_weight += weight;
                }

                return sum_voltage / sum_weight;
            };

            // 步骤 3：用已知帧头估计判决电平。
            float sum_high = 0, sum_low = 0;
            int count_high = 0, count_low = 0;

            for (size_t bit = 0; bit < d_symbol_kernel.size(); ++bit)
            {
                float val = symbol_avg(consumed + (int)bit * d_sps);
                if (d_symbol_kernel[bit] > 0)
                {
                    sum_high += val;
                    count_high++;
                }
                else
                {
                    sum_low += val;
                    count_low++;
                }
            }

            float high_level = (count_high > 0) ? (sum_high / count_high) : 1.0f;
            float low_level = (count_low > 0) ? (sum_low / count_low) : -1.0f;
            float alpha = 0.05f;

            std::vector<uint8_t> packet_bytes_vec(d_packet_bytes, 0);

            // 步骤 4：逐符号判决，并更新高低电平。
            for (int b = 0; b < d_packet_bits; ++b)
            {
                int bit_start = consumed + b * d_sps;
                float avg_voltage = symbol_avg(bit_start);

                float threshold = (high_level + low_level) / 2.0f;
                uint8_t bit_val = (avg_voltage > threshold) ? 1 : 0;

                if (bit_val)
                {
                    high_level = (1.0f - alpha) * high_level + alpha * avg_voltage;
                }
                else
                {
                    low_level = (1.0f - alpha) * low_level + alpha * avg_voltage;
                }

                int byte_idx = b / 8;
                packet_bytes_vec[byte_idx] |= (bit_val << (7 - (b % 8)));
            }

            // 步骤 5：校验帧头，合格后输出末尾 15 字节。
            uint8_t access_code[] = {0x16, 0xE8, 0xD3, 0x77, 0x15, 0x1C, 0x71, 0x2D};
            int header_len = 8;
            int max_header_errors = 2;
            int header_errors = max_header_errors + 1;

            if (d_packet_bytes >= header_len)
            {
                header_errors = 0;
                for (int i = 0; i < header_len; ++i)
                {
                    uint8_t diff = packet_bytes_vec[i] ^ access_code[i];
                    for (int bit = 0; bit < 8; ++bit)
                    {
                        header_errors += (diff >> bit) & 1;
                    }
                }
            }

            if (header_errors <= max_header_errors)
            {
                int offset = d_packet_bytes - output_len;
                if (offset >= 0)
                {
                    std::memcpy(&out[produced], packet_bytes_vec.data() + offset, output_len);
                    produced += output_len;
                    good_count++;
                }
            }
            else
            {
                bad_count++;
                d_invalid_packets++;
                // if (d_invalid_packets % 10 == 0)
                // {
                //     int print_len = std::min(header_len, d_packet_bytes);
                //     std::printf("Noise Bad Header len=%d errors=%d:", print_len, header_errors);
                //     for (int i = 0; i < print_len; ++i)
                //     {
                //         std::printf(" %02X", packet_bytes_vec[i]);
                //     }
                //     std::printf("\n");
                // }
            }
            find_count++;

            // 跳过当前帧，继续搜帧。
            consumed += d_payload_samples;
            d_state = SEARCH;
        }
    }
    long long current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (current_time - last_time > 1000)
    {
        std::cout << "Noise Find: " << find_count
                  << ", Good: " << good_count
                  << ", Bad: " << bad_count << std::endl;
        find_count = 0;
        good_count = 0;
        bad_count = 0;
        last_time = current_time;
    }

    consume_each(consumed);
    return produced;
}

} /* namespace robomaster */
} /* namespace gr */
