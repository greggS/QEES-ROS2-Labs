#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp" 
#include "std_msgs/msg/bool.hpp" 


using std::placeholders::_1;

// This node acts as the car driver that sets a desired car speed 
class TestController : public rclcpp::Node
{
public:
  TestController() : Node("test_controller")
  {
    desired_speed_ = 3;

    // Checking if the test has passed or failed to stop the car after it's finished
    test_result_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "test_result", 
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
      std::bind(&TestController::test_result_callback, this, _1));

    // Creating the publisher for the desired speed
    desired_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>("desired_speed", 
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()));

    // The desired speed should be published continuously 
    desired_speed_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TestController::send_desired_speed, this));
  }

  void test_result_callback(const std_msgs::msg::Bool::SharedPtr _msg) 
  {
    desired_speed_ = 0;
  }

  void send_desired_speed()
  {
    auto msg = std::make_unique<std_msgs::msg::Float32>();
    msg->data = 0.5;

    desired_speed_publisher_->publish(std::move(msg));
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr test_result_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_speed_publisher_;
  rclcpp::TimerBase::SharedPtr desired_speed_timer_;

private:
  float desired_speed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestController>());
  rclcpp::shutdown();
  return 0;
}