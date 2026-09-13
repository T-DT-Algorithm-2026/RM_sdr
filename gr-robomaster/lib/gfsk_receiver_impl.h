/* -*- c++ -*- */
/*
 * Copyright 2026 liu.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ROBOMASTER_GFSK_RECEIVER_IMPL_H
#define INCLUDED_ROBOMASTER_GFSK_RECEIVER_IMPL_H

#include <gnuradio/robomaster/gfsk_receiver.h>
#include <vector>

namespace gr {
namespace robomaster {

class gfsk_receiver_impl : public gfsk_receiver
{
private:
    // 接收参数。
    float d_threshold;
    int d_sps;
    int d_packet_bytes;

    // 帧长与相关模板。
    int d_packet_bits;
    int d_payload_samples;
    std::vector<float> d_kernel;
    std::vector<int> d_symbol_kernel;
    std::vector<int> d_sign_prefix;
    int d_kernel_len;

    // 搜帧与提取状态。
    enum State { SEARCH, EXTRACT };
    State d_state;

    // 帧头位置。
    int d_start_idx;

    // 运行统计。
    int find_count = 0;
    int good_count = 0;
    int bad_count = 0;
    long long last_time = 0;
    int d_invalid_packets = 0;

public:
    gfsk_receiver_impl(float threshold, int sps, int packet_bytes);
    ~gfsk_receiver_impl();

    // 声明搜帧和提取所需的输入量。
    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    // 执行接收流程。
    int general_work(int noutput_items,
                     gr_vector_int &ninput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items);
};

} // namespace robomaster
} // namespace gr

#endif /* INCLUDED_ROBOMASTER_GFSK_RECEIVER_IMPL_H */
