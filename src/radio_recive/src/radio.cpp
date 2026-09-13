#include "radio.h"
#include "crc.h"

#pragma pack(1)

// 帧头 (固定 5 字节)
struct FrameHeader {
    uint8_t  sof;
    uint16_t dataLength;
    uint8_t  seq;
    uint8_t  crc8;
};

constexpr size_t kRobotPositionCount = 6;
constexpr size_t kRobotHpCount = 6;
constexpr size_t kFireCount = 5;
constexpr size_t kBuffCount = 5;
constexpr size_t kPasswordLength = 6;
constexpr size_t kRobotPositionPayloadSize = kRobotPositionCount * 2 * sizeof(uint16_t);
constexpr size_t kRobotHpPayloadSize = kRobotHpCount * sizeof(uint16_t);
constexpr size_t kFirePayloadSize = kFireCount * sizeof(uint16_t);
constexpr size_t kBuffEntrySize = 7;
constexpr size_t kBuffEntriesPayloadSize = kBuffCount * kBuffEntrySize;
constexpr size_t kBuffPayloadSize = kBuffEntriesPayloadSize + 1 + kBuffCount;

struct StateStatus {
    uint32_t supply_area_occupied : 1;           // bit 0
    uint32_t central_highland_status : 2;        // bit 1-2
    uint32_t trapezoid_highland_occupied : 1;    // bit 3
    uint32_t sentry_patrol_status : 2;           // bit 4-5
    uint32_t front_sentry_gain_status : 2;       // bit 6-7
    uint32_t base_gain_occupied : 1;             // bit 8
    uint32_t enemy_front_channel_detected : 1;   // bit 9
    uint32_t enemy_back_channel_detected : 1;    // bit 10
    uint32_t own_front_channel_detected : 1;     // bit 11
    uint32_t own_back_channel_detected : 1;      // bit 12
    uint32_t highland_upper_detected : 1;        // bit 13
    uint32_t flying_slope_back_detected : 1;     // bit 14
    uint32_t highway_upper_detected : 1;         // bit 15
    uint32_t reserved : 16;
};

struct StateData {
    uint16_t remaining_coin;
    uint16_t total_coin;
    StateStatus status;
};

static uint16_t read_uint16_le(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}
#pragma pack()

namespace radio{

Receiver::Receiver(const rclcpp::NodeOptions& node_options)
: Node("radio_node", node_options),
  zmq_context_(1),
  zmq_subscriber_(zmq_context_, zmq::socket_type::sub),
  zmq_noise_subscriber_1_(zmq_context_, zmq::socket_type::sub),
  zmq_noise_subscriber_2_(zmq_context_, zmq::socket_type::sub),
  zmq_noise_subscriber_3_(zmq_context_, zmq::socket_type::sub)
{
    RCLCPP_INFO(this->get_logger(), "radio_node start");
    // 1. 初始化 ROS 发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("/radio", 10);
    robot_position_publisher_ = this->create_publisher<radio_interface::msg::Position>("robot_position", 10);
    robot_hp_publisher_ = this->create_publisher<radio_interface::msg::Hp>("radio_hp", 10);
    fire_publisher_ = this->create_publisher<radio_interface::msg::Fire>("radio_fire", 10);
    state_publisher_ = this->create_publisher<radio_interface::msg::State>("radio_state", 10);
    buff_publisher_ = this->create_publisher<radio_interface::msg::Buff>("radio_buff", 10);
    password_publisher_ = this->create_publisher<radio_interface::msg::Password>("key_usart_sender", 10);

    // 2. 配置 ZMQ 接收端 (连接到 GNU Radio 发送的端口)
    zmq_subscriber_.connect("tcp://127.0.0.1:5555");
    zmq_subscriber_.set(zmq::sockopt::subscribe, ""); // 订阅所有主题
    zmq_noise_subscriber_1_.connect("tcp://127.0.0.1:6666");
    zmq_noise_subscriber_1_.set(zmq::sockopt::subscribe, "");
    zmq_noise_subscriber_2_.connect("tcp://127.0.0.1:6667");
    zmq_noise_subscriber_2_.set(zmq::sockopt::subscribe, "");
    zmq_noise_subscriber_3_.connect("tcp://127.0.0.1:6668");
    zmq_noise_subscriber_3_.set(zmq::sockopt::subscribe, "");
    rm_buffer.reserve(512);
    rm_noise_1.reserve(512);
    rm_noise_2.reserve(512);
    rm_noise_3.reserve(512);

    RCLCPP_INFO(this->get_logger(), "C++ SDR Bridge Node 启动，正在高速监听 5555、6666、6667、6668 端口...");

    data_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    noise_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 3. 创建高频定时器 (例如 1000Hz = 1ms)，非阻塞拉取数据
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&Receiver::receive_data_callback, this),
        data_callback_group_);

    noise_timer_1_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&Receiver::receive_noise_1_callback, this),
        noise_callback_group_);

    noise_timer_2_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&Receiver::receive_noise_2_callback, this),
        noise_callback_group_);

    noise_timer_3_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&Receiver::receive_noise_3_callback, this),
        noise_callback_group_);

}

void Receiver::receive_data_callback()
{
    receive_from_socket(zmq_subscriber_, rm_buffer, "data:5555");
}

void Receiver::receive_noise_1_callback()
{
    receive_from_socket(zmq_noise_subscriber_1_, rm_noise_1, "noise1:6666");
}

void Receiver::receive_noise_2_callback()
{
    receive_from_socket(zmq_noise_subscriber_2_, rm_noise_2, "noise2:6667");
}

void Receiver::receive_noise_3_callback()
{
    receive_from_socket(zmq_noise_subscriber_3_, rm_noise_3, "noise3:6668");
}

void Receiver::record_frame_rate(bool is_position_frame)
{
    const auto now = std::chrono::steady_clock::now();
    double position_fps = 0.0;
    double password_fps = 0.0;
    bool should_report = false;

    {
        std::lock_guard<std::mutex> lock(frame_rate_mutex_);
        if (!frame_rate_window_started_) {
            frame_rate_window_start_ = now;
            frame_rate_window_started_ = true;
        }

        if (is_position_frame) {
            ++position_frame_count_;
        } else {
            ++password_frame_count_;
        }

        const double elapsed_seconds =
            std::chrono::duration<double>(now - frame_rate_window_start_).count();
        if (elapsed_seconds >= 1.0) {
            position_fps = static_cast<double>(position_frame_count_) / elapsed_seconds;
            password_fps = static_cast<double>(password_frame_count_) / elapsed_seconds;
            position_frame_count_ = 0;
            password_frame_count_ = 0;
            frame_rate_window_start_ = now;
            should_report = true;
        }
    }

    if (should_report) {
        RCLCPP_INFO(
            this->get_logger(),
            "FrameRate: position=%.2f FPS, password=%.2f FPS",
            position_fps,
            password_fps);
    }
}

void Receiver::receive_from_socket(zmq::socket_t& subscriber, std::vector<uint8_t>& buffer, const char* source_name)
{
    zmq::message_t message;
    // 使用 ZMQ_DONTWAIT 实现非阻塞拉取
    auto res = subscriber.recv(message, zmq::recv_flags::dontwait);

    if (res.has_value()) {
        // 1. 获取指针和长度
        const unsigned char* data_ptr = static_cast<const unsigned char*>(message.data());
        size_t data_len = message.size();

        buffer.insert(buffer.end(), data_ptr, data_ptr + data_len);
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "RECV[%s]: recv_len=%zu, buffer_size=%zu",
        //     source_name,
        //     data_len,
        //     buffer.size());
        process_buffer(buffer, source_name);


        // 2. 预先分配好固定长度的字符串（极其关键：一次性分配，无碎片）
        std::string hex_str(data_len * 2, '0'); 

        // 3. 静态查表，纯位运算（CPU 执行速度极快）
        constexpr char hex_chars[] = "0123456789ABCDEF";
        for(size_t i = 0; i < data_len; ++i) {
            hex_str[i * 2]     = hex_chars[(data_ptr[i] >> 4) & 0x0F]; // 取高 4 位
            hex_str[i * 2 + 1] = hex_chars[data_ptr[i] & 0x0F];        // 取低 4 位
        }
        
        // 3. 将肉眼可见的 16 进制字符串打包成 ROS 2 消息发出
        auto msg = std_msgs::msg::String();
        msg.data = hex_str; 
        publisher_->publish(msg);
        
        // 4. 【关键】改为 RCLCPP_INFO，让它强制在终端显形！
        // RCLCPP_INFO(this->get_logger(), "✅ 成功截获 SDR 数据! 长度: %zu 字节, 内容: %s", data_len, hex_str.c_str());
    }
    
    // 如果 res 没有 value，说明当前端口没新数据，直接退出回调，不浪费 CPU
}


void Receiver::process_buffer(std::vector<uint8_t>& buffer, const char* source_name)
{
    // 修改：只要有 5 个字节就可以校验包头了
    while(buffer.size() >= 5)
    {
        if(buffer[0] != 0xA5)
        {
            // RCLCPP_WARN(
            //     this->get_logger(),
            //     "DROP[%s]: bad SOF, byte=0x%02X, buffer_size=%zu",
            //     source_name,
            //     static_cast<unsigned>(buffer[0]),
            //     buffer.size());
            buffer.erase(buffer.begin());
            continue;
        }
        uint8_t* pack_ptr = buffer.data(); // 拿到包首地址

        // 1. 优先校验前 5 个字节的 CRC8
        // 只有校验通过，才能信任里面的 dataLength
        if (!Verify_CRC8_Check_Sum(pack_ptr, 5)) {
            // CRC8 失败：这个 0xA6 是伪造的，或者数据错了。扔掉开头的 0xA6 重新找。
            // RCLCPP_WARN(
            //     this->get_logger(),
            //     "DROP[%s]: CRC8 failed, header=%02X %02X %02X %02X %02X, buffer_size=%zu",
            //     source_name,
            //     static_cast<unsigned>(pack_ptr[0]),
            //     static_cast<unsigned>(pack_ptr[1]),
            //     static_cast<unsigned>(pack_ptr[2]),
            //     static_cast<unsigned>(pack_ptr[3]),
            //     static_cast<unsigned>(pack_ptr[4]),
            //     buffer.size());
            buffer.erase(buffer.begin());
            RCLCPP_WARN(this->get_logger(), "CRC8 校验失败，丢弃错包！");
            continue;
        }

        // CRC8 通过后，再计算和信任包长
        FrameHeader* header = reinterpret_cast<FrameHeader*>(buffer.data());
        int pack_len = 5 + header->dataLength + 2 + 2;

        // 加入合理的包长限制，防止算出一个巨大包长导致死等
        if (pack_len > 512 || pack_len < 9) {
            // RCLCPP_WARN(
            //     this->get_logger(),
            //     "DROP[%s]: invalid packet length, dataLength=%u, pack_len=%d",
            //     source_name,
            //     static_cast<unsigned>(header->dataLength),
            //     pack_len);
            buffer.erase(buffer.begin());
            continue;
        }

        // 判断缓存区数据是否已经凑齐整包
        if(buffer.size() < static_cast<size_t>(pack_len))
        {
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "WAIT[%s]: packet incomplete, need=%d, have=%zu, dataLength=%u",
            //     source_name,
            //     pack_len,
            //     buffer.size(),
            //     static_cast<unsigned>(header->dataLength));
            return; // 凑不齐则等待下一次接收
        }

        // 2. 校验整包的 CRC16
        if (!Verify_CRC16_Check_Sum(pack_ptr, pack_len)) {
            // CRC16 失败：包头对了，但数据段被干扰了。这整包彻底废了，全部删掉。
            // RCLCPP_WARN(
            //     this->get_logger(),
            //     "DROP[%s]: CRC16 failed, cmd_id=0x%04X, pack_len=%d, dataLength=%u",
            //     source_name,
            //     static_cast<unsigned>(cmd_id),
            //     pack_len,
            //     static_cast<unsigned>(header->dataLength));
            buffer.erase(buffer.begin(), buffer.begin() + 5);
            // RCLCPP_WARN(this->get_logger(), "CRC16 校验失败，丢弃错包！");
            continue;
        }
        uint16_t cmd_id = (pack_ptr[6] << 8) | pack_ptr[5];
        const uint8_t* data_ptr = pack_ptr + 7;
        const size_t data_len = header->dataLength;
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "PASS[%s]: CRC OK, cmd_id=0x%04X, data_len=%zu, pack_len=%d",
        //     source_name,
        //     static_cast<unsigned>(cmd_id),
        //     data_len,
        //     pack_len);
        switch(cmd_id)
        {
            case 0x0A01: {
                if (data_len < kRobotPositionPayloadSize) {
                    RCLCPP_WARN(this->get_logger(), "DROP[%s]: 0x0A01 data too short, data_len=%zu, need=%zu", source_name, data_len, kRobotPositionPayloadSize);
                    break;
                }
                auto position_msg = radio_interface::msg::Position();
                for (size_t i = 0; i < position_msg.x.size(); ++i) {
                    const size_t offset = i * 4;
                    position_msg.x[i] = read_uint16_le(data_ptr + offset);
                    position_msg.y[i] = read_uint16_le(data_ptr + offset + 2);
                }
                robot_position_publisher_->publish(position_msg);
                record_frame_rate(true);
                break;
            }
            case 0x0A02: {
                if (data_len < kRobotHpPayloadSize) {
                    RCLCPP_WARN(this->get_logger(), "DROP[%s]: 0x0A02 data too short, data_len=%zu, need=%zu", source_name, data_len, kRobotHpPayloadSize);
                    break;
                }
                auto hp_msg = radio_interface::msg::Hp();
                for (size_t i = 0; i < hp_msg.hp.size(); ++i) {
                    hp_msg.hp[i] = read_uint16_le(data_ptr + i * 2);
                }
                robot_hp_publisher_->publish(hp_msg);
                break;
            }
            case 0x0A03: {
                if (data_len < kFirePayloadSize) {
                    RCLCPP_WARN(this->get_logger(), "DROP[%s]: 0x0A03 data too short, data_len=%zu, need=%zu", source_name, data_len, kFirePayloadSize);
                    break;
                }
                auto fire_msg = radio_interface::msg::Fire();
                for (size_t i = 0; i < fire_msg.fire.size(); ++i) {
                    fire_msg.fire[i] = read_uint16_le(data_ptr + i * 2);
                }
                fire_publisher_->publish(fire_msg);
                // RCLCPP_INFO(
                //     this->get_logger(),
                //     "Fire: hero=%u, infantry3=%u, infantry4=%u, aerial=%u, sentry=%u",
                //     static_cast<unsigned>(fire_msg.fire[0]),
                //     static_cast<unsigned>(fire_msg.fire[1]),
                //     static_cast<unsigned>(fire_msg.fire[2]),
                //     static_cast<unsigned>(fire_msg.fire[3]),
                //     static_cast<unsigned>(fire_msg.fire[4]));
                break;
            }
            case 0x0A04: {
                if (data_len < sizeof(StateData)) {
                    RCLCPP_WARN(this->get_logger(), "DROP[%s]: 0x0A04 data too short, data_len=%zu, need=%zu", source_name, data_len, sizeof(StateData));
                    break;
                }
                const auto* state_data = reinterpret_cast<const StateData*>(data_ptr);
                auto state_msg = radio_interface::msg::State();
                state_msg.remaining_coin = state_data->remaining_coin;
                state_msg.total_coin = state_data->total_coin;
                state_msg.supply_area_occupied = state_data->status.supply_area_occupied;
                state_msg.central_highland_status = state_data->status.central_highland_status;
                state_msg.trapezoid_highland_occupied = state_data->status.trapezoid_highland_occupied;
                state_msg.sentry_patrol_status = state_data->status.sentry_patrol_status;
                state_msg.front_sentry_gain_status = state_data->status.front_sentry_gain_status;
                state_msg.base_gain_occupied = state_data->status.base_gain_occupied;
                state_msg.enemy_front_channel_detected = state_data->status.enemy_front_channel_detected;
                state_msg.enemy_back_channel_detected = state_data->status.enemy_back_channel_detected;
                state_msg.own_front_channel_detected = state_data->status.own_front_channel_detected;
                state_msg.own_back_channel_detected = state_data->status.own_back_channel_detected;
                state_msg.highland_upper_detected = state_data->status.highland_upper_detected;
                state_msg.flying_slope_back_detected = state_data->status.flying_slope_back_detected;
                state_msg.highway_upper_detected = state_data->status.highway_upper_detected;
                state_publisher_->publish(state_msg);
                // RCLCPP_INFO(
                //     this->get_logger(),
                //     "State: remaining_coin=%u, total_coin=%u, supply=%u, central_highland=%u, trapezoid=%u, sentry_patrol=%u, front_sentry=%u, base_gain=%u, enemy_front=%u, enemy_back=%u, own_front=%u, own_back=%u, highland_upper=%u, flying_slope_back=%u, highway_upper=%u",
                //     static_cast<unsigned>(state_data->remaining_coin),
                //     static_cast<unsigned>(state_data->total_coin),
                //     static_cast<unsigned>(state_data->status.supply_area_occupied),
                //     static_cast<unsigned>(state_data->status.central_highland_status),
                //     static_cast<unsigned>(state_data->status.trapezoid_highland_occupied),
                //     static_cast<unsigned>(state_data->status.sentry_patrol_status),
                //     static_cast<unsigned>(state_data->status.front_sentry_gain_status),
                //     static_cast<unsigned>(state_data->status.base_gain_occupied),
                //     static_cast<unsigned>(state_data->status.enemy_front_channel_detected),
                //     static_cast<unsigned>(state_data->status.enemy_back_channel_detected),
                //     static_cast<unsigned>(state_data->status.own_front_channel_detected),
                //     static_cast<unsigned>(state_data->status.own_back_channel_detected),
                //     static_cast<unsigned>(state_data->status.highland_upper_detected),
                //     static_cast<unsigned>(state_data->status.flying_slope_back_detected),
                //     static_cast<unsigned>(state_data->status.highway_upper_detected));
                break;
            }
            case 0x0A05: {
                if (data_len < kBuffPayloadSize) {
                    RCLCPP_WARN(this->get_logger(), "DROP[%s]: 0x0A05 data too short, data_len=%zu, need=%zu", source_name, data_len, kBuffPayloadSize);
                    break;
                }
                auto buff_msg = radio_interface::msg::Buff();
                for (size_t i = 0; i < kBuffCount; ++i) {
                    const size_t offset = i * kBuffEntrySize;
                    buff_msg.heal[i] = data_ptr[offset];
                    buff_msg.cooldown[i] = read_uint16_le(data_ptr + offset + 1);
                    buff_msg.defence[i] = data_ptr[offset + 3];
                    buff_msg.undefence[i] = data_ptr[offset + 4];
                    buff_msg.attack[i] = read_uint16_le(data_ptr + offset + 5);
                }
                buff_msg.sentry_posture = data_ptr[kBuffEntriesPayloadSize];
                for (size_t i = 0; i < kBuffCount; ++i) {
                    buff_msg.main_posture[i] = data_ptr[kBuffEntriesPayloadSize + 1 + i];
                }
                buff_publisher_->publish(buff_msg);
                break;
            }
            case 0x0A06: {
                if (data_len < kPasswordLength) {
                    RCLCPP_WARN(this->get_logger(), "DROP[%s]: 0x0A06 data too short, data_len=%zu, need=%zu", source_name, data_len, kPasswordLength);
                    break;
                }
                auto password_msg = radio_interface::msg::Password();
                for (size_t i = 0; i < password_msg.password.size(); ++i) {
                    password_msg.password[i] = data_ptr[i];
                }
                password_publisher_->publish(password_msg);
                record_frame_rate(false);
                break;
            }
            default:
                RCLCPP_WARN(
                    this->get_logger(),
                    "SKIP[%s]: unhandled cmd_id=0x%04X, data_len=%zu",
                    source_name,
                    static_cast<unsigned>(cmd_id),
                    data_len);
                break;
        }
        buffer.erase(buffer.begin(), buffer.begin() + pack_len);

    }
}


}//namespace radio

RCLCPP_COMPONENTS_REGISTER_NODE(radio::Receiver)
