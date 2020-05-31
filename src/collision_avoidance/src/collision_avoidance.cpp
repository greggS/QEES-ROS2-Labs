#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <string>
#include <unistd.h>
#include <time.h>
#include <math.h>

using std::placeholders::_1;

class Collision_avoidance : public rclcpp::Node
{
public:
	Collision_avoidance() : Node("collision_avoidance"
)	{
		// Subscribe to classification data 
  		markov_data_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>("/markov_prob", 
			 rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
			std::bind(&Collision_avoidance::collision_calculation, this, _1));

  		// Advertise probability data
    	collision_pub = this->create_publisher<std_msgs::msg::Float32>("/collision_probability", rclcpp::SystemDefaultsQoS());		
	}

private:

	void collision_calculation(const std_msgs::msg::Float32MultiArray::SharedPtr markov_msg)
	{
		uint rows = markov_msg->layout.dim[0].size;
		uint cols = markov_msg->layout.dim[1].size;

		//Construct output message
		auto collision_output_msg = std_msgs::msg::Float32();
		collision_output_msg.data = 0.5;

		//------ WRITE YOUR CODE HERE -------//

		//The markov message can be parsed in the same way as the classification message

		// -------- TO HERE ---------//

		//publish collision probability
		collision_pub->publish(collision_output_msg);

	}
	
  	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr collision_pub;
  	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr markov_data_subscription;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Collision_avoidance>());
	rclcpp::shutdown();

	return 0;
}