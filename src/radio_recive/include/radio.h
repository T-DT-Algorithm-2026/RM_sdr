#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "radio_interface/msg/buff.hpp"
#include "radio_interface/msg/fire.hpp"
#include "radio_interface/msg/hp.hpp"
#include "radio_interface/msg/password.hpp"
#include "radio_interface/msg/position.hpp"
#include "radio_interface/msg/state.hpp"
#include <std_msgs/msg/string.hpp>
#include <zmq.hpp> // ZMQ 的 C++ 接口
#include <chrono>
#include <iostream>
#include <mutex>
#include <sstream>
#include <vector>

namespace radio{
class Receiver : public rclcpp::Node
{
    public:
    Receiver(const rclcpp::NodeOptions& node_options);
    ~Receiver(){}
    
    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<radio_interface::msg::Position>::SharedPtr robot_position_publisher_;
    rclcpp::Publisher<radio_interface::msg::Hp>::SharedPtr robot_hp_publisher_;
    rclcpp::Publisher<radio_interface::msg::Fire>::SharedPtr fire_publisher_;
    rclcpp::Publisher<radio_interface::msg::State>::SharedPtr state_publisher_;
    rclcpp::Publisher<radio_interface::msg::Buff>::SharedPtr buff_publisher_;
    rclcpp::Publisher<radio_interface::msg::Password>::SharedPtr password_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr noise_timer_1_;
    rclcpp::TimerBase::SharedPtr noise_timer_2_;
    rclcpp::TimerBase::SharedPtr noise_timer_3_;
    rclcpp::CallbackGroup::SharedPtr data_callback_group_;
    rclcpp::CallbackGroup::SharedPtr noise_callback_group_;

    void receive_data_callback();
    void receive_noise_1_callback();
    void receive_noise_2_callback();
    void receive_noise_3_callback();
    void receive_from_socket(zmq::socket_t& subscriber, std::vector<uint8_t>& buffer, const char* source_name);
    void process_buffer(std::vector<uint8_t>& buffer, const char* source_name);
    void record_frame_rate(bool is_position_frame);
    
    zmq::context_t zmq_context_;
    zmq::socket_t zmq_subscriber_;
    zmq::socket_t zmq_noise_subscriber_1_;
    zmq::socket_t zmq_noise_subscriber_2_;
    zmq::socket_t zmq_noise_subscriber_3_;
    std::vector<uint8_t> rm_buffer;
    std::vector<uint8_t> rm_noise_1;
    std::vector<uint8_t> rm_noise_2;
    std::vector<uint8_t> rm_noise_3;
    std::mutex frame_rate_mutex_;
    std::chrono::steady_clock::time_point frame_rate_window_start_;
    size_t position_frame_count_{0};
    size_t password_frame_count_{0};
    bool frame_rate_window_started_{false};
};
}//namespace radio
