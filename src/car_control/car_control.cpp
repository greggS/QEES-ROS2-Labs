#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp" 
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class CarControl : public rclcpp::Node
{
public:
	CarControl() : Node("car_control")
	{
    collision_probability_ = 1;
    desired_speed_ = 0;

    // Subscribe to the collision probability topic
		collision_probability_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
			"/collision_probability", 
			rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
			std::bind(&CarControl::collision_probability_callback, this, _1));
   
    // Subscribe to the desired speed topic 
    desired_speed_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "desired_speed", 
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
      std::bind(&CarControl::desired_speed_callback, this, _1));

    // Create the final car control commands publisher
    command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_demo", 
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()));

    // If a message is to be sent right after a node is created it has to be done with a slight delay
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CarControl::execute_command, this));
	}

	void collision_probability_callback(const std_msgs::msg::Float32::SharedPtr _msg) 
	{
    collision_probability_ = _msg->data;
	}

  void desired_speed_callback(const std_msgs::msg::Float32::SharedPtr _msg) 
  {
    desired_speed_ = _msg->data;
  }

  void execute_command()
  {
    float command = calculate_command();
    send_command(command);
  }

  float calculate_command()
  {
    return desired_speed_;
  }

  void send_command(float command)
  {
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    // linear.x means the speed value
    cmd_msg->linear.x = command;
    // compensation for an unexpected drift
    cmd_msg->angular.z = -0.004;

    command_publisher_->publish(std::move(cmd_msg));
  }

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr collision_probability_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_speed_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  float collision_probability_;
  float desired_speed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarControl>());
  rclcpp::shutdown();
  return 0;
}