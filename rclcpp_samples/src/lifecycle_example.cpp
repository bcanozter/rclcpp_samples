
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"

class LifecycleExample : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleExample()
      : rclcpp_lifecycle::LifecycleNode("lifecycle_node")
  {
    // Manually trigger the transition here or
    // remotely by`ros2 service call /lifecycle_node/change_state lifecycle_msgs/srv/ChangeState`
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    // this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    // this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    // this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }

  void publish()
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle HelloWorld #" + std::to_string(++_count);
    if (!pub_->is_activated())
    {
      RCLCPP_INFO(
          get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    }
    else
    {
      RCLCPP_INFO(
          get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
    }
    pub_->publish(std::move(msg));
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), [this]()
        { return this->publish(); });

    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_activate(state);

    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {

    timer_.reset();
    pub_.reset();

    RCLCPP_INFO(get_logger(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
  {
    timer_.reset();
    pub_.reset();

    RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.", state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  int _count = 0;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<LifecycleExample> node =
      std::make_shared<LifecycleExample>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}