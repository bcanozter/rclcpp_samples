#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
class CustomQosPublisher : public rclcpp::Node
{
public:
  CustomQosPublisher()
      : Node("custom_qos_publisher")
  {
    // Method 1:
    //! History (Depth) is reported UNKNOWN for rmw_fastrtps
    rclcpp::QoS custom_qos_profile(
        rclcpp::QoSInitialization(
            rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1)); // History depth of 1 messages
    /*
    Options
      BestEffort = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      Reliable = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      SystemDefault = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
      BestAvailable = RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE,
      Unknown = RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
    */
    // Ref: https://github.com/ros2/rclcpp/blob/b7e4aad091c6b334b9ad17a8f83007ea9901cc3b/rclcpp/include/rclcpp/qos.hpp#L42
    custom_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort); // or RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    custom_qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
    custom_qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
    custom_qos_profile.liveliness_lease_duration(std::chrono::seconds(1));

    // Method 2:
    rclcpp::QoS sensor_qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    // override the default sensor data qos
    sensor_qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", custom_qos_profile);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CustomQosPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(_count++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  int _count = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomQosPublisher>());
  rclcpp::shutdown();
  return 0;
}