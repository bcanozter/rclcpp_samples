#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class CustomQosSubscriber : public rclcpp::Node
{
public:
CustomQosSubscriber()
  : Node("custom_qos_subscriber")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    // Now you can override depth and history
    qos_profile.keep_last(2); // or .keep_all() for KEEP_ALL: https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/qos.hpp#L178
    qos_profile.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST); // or KEEP_ALL
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", qos_profile, std::bind(&CustomQosSubscriber::topic_callback, this, placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomQosSubscriber>());
  rclcpp::shutdown();
  return 0;
}