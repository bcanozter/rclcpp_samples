#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class DynamicQosSubscriber : public rclcpp::Node
{
public:
    DynamicQosSubscriber()
        : Node("dynamic_qos_subscriber")
    {
        _create_subscription();
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&DynamicQosSubscriber::timer_callback, this));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        (void)msg;
    }
    void timer_callback()
    {
        std::vector<rclcpp::TopicEndpointInfo> info;
        info = this->get_publishers_info_by_topic("/topic");
        if (info.size() > 0)
        {
            rclcpp::QoS qos_profile = info.front().qos_profile();
            rmw_qos_profile_t &rmw_qos_profile = info.front().qos_profile().get_rmw_qos_profile();
            rclcpp::QoSCheckCompatibleResult compatibility_result =
                rclcpp::qos_check_compatible(qos_profile, subscription_qos_profile);
            if (compatibility_result.compatibility == rclcpp::QoSCompatibility::Error)
            {
                RCLCPP_ERROR(rclcpp::get_logger("qos_check"), "Incompatible QoS policies detected: %s",
                             compatibility_result.reason.c_str());
                subscription_qos_profile.reliability(rmw_qos_profile.reliability);
                subscription_qos_profile.liveliness(rmw_qos_profile.liveliness);
                reset_subscription();
            }
            // else if (compatibility_result.compatibility == rclcpp::QoSCompatibility::Warning)
            // {
            //     RCLCPP_WARN(rclcpp::get_logger("qos_check"), "QoS compatibility warning: %s",
            //                 compatibility_result.reason.c_str());
            //     subscription_qos_profile.reliability(rmw_qos_profile.reliability);
            //     subscription_qos_profile.liveliness(rmw_qos_profile.liveliness);
            //     reset_subscription();
            // }
            // else
            // {
            //     RCLCPP_INFO(rclcpp::get_logger("qos_check"), "QoS compatibility OK");
            // }
            if (!subscription_)
            {
                _create_subscription();
            }
            // RCLCPP_DEBUG(this->get_logger(),
            //             "QoS Profile - history: %d, depth: %zu, reliability: %d, durability: %d, deadline: %ld ns, lifespan: %ld ns, liveliness: %d, liveliness_lease_duration: %ld ns",
            //             qos_profile.history, qos_profile.depth, qos_profile.reliability, qos_profile.durability,
            //             qos_profile.deadline.nsec, qos_profile.lifespan.nsec,
            //             qos_profile.liveliness, qos_profile.liveliness_lease_duration.nsec);
        }
    }

    void _create_subscription(void)
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", subscription_qos_profile, std::bind(&DynamicQosSubscriber::topic_callback, this, placeholders::_1));
    }

    void reset_subscription(void)
    {
        subscription_.reset();
        subscription_ = NULL;
    }
    rclcpp::QoS subscription_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicQosSubscriber>());
    rclcpp::shutdown();
    return 0;
}