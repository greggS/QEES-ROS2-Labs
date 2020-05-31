#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp" 
#include "std_msgs/msg/bool.hpp" 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sys/time.h>

using std::placeholders::_1;

#define TEST_PASS_ODOMETRY_DISTANCE 100
#define TEST_FAILE_SAFETY_DISTANCE 0.3

#define EFFICIENCY_WEIGHT 0.2
#define SAFETY_WEIGHT 0.8

#define OPTIMAL_TIME 70.0
#define OPTIMAL_SAFETY_DISTANCE 1.0

class TestScore : public rclcpp::Node
{
public:
	TestScore() : Node("test_score")
	{
		current_odometry_ = 0;
		current_minimum_distance_ = 999;
		current_test_time_ = 0;
		current_score_ = 0;

    // Test time start
		start_time_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // Subscribe to car trip meter
		odometry_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
			"/demo/distance_demo", 
			rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
			std::bind(&TestScore::odometry_subscription_callback, this, _1));
	
    // Subscribe to Lidar data
  	laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      	"/ray/laserscan", 
      	rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
      	std::bind(&TestScore::laser_subscription_callback, this, _1));
   
    // Create the final test result publisher
    test_result_publisher_ = this->create_publisher<std_msgs::msg::Bool>("test_result", 
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()));
	}

	void odometry_subscription_callback(const std_msgs::msg::Float32::SharedPtr _msg) 
	{
		current_odometry_ = _msg->data;
		
		check_test_passed();
	}

	void laser_subscription_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
		float min_range = _msg->range_max + 1;
    for (auto i = 0u; i < _msg->ranges.size(); ++i) {
      auto range = _msg->ranges[i];
      if (range > _msg->range_min && range < _msg->range_max && range < min_range) {
        	min_range = range;
      }
    }  		

    if (min_range < current_minimum_distance_) {
    	current_minimum_distance_ = min_range;
    }
    calculate_score();
    check_safety_test_failed();
  }

  // calculating the final test score
  void calculate_score() 
  {
  	unsigned long now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  	current_test_time_ = now_time - start_time_;
  	float efficiency_score = 1;
  	if (current_test_time_ > OPTIMAL_TIME) {
  		efficiency_score = OPTIMAL_TIME / current_test_time_;
  	}

  	float safety_score = 1;
  	if (current_minimum_distance_ < OPTIMAL_SAFETY_DISTANCE) {
  		safety_score = current_minimum_distance_ / OPTIMAL_SAFETY_DISTANCE;
  	}

  	RCLCPP_INFO(this->get_logger(), "time: %d, efficiency_score: %f", current_test_time_, efficiency_score);

  	current_score_ = EFFICIENCY_WEIGHT * efficiency_score + SAFETY_WEIGHT * safety_score;
  }

  void check_safety_test_failed() 
  {
    if (current_minimum_distance_ <= TEST_FAILE_SAFETY_DISTANCE) {
      RCLCPP_INFO(this->get_logger(), "Test failed!");

      auto test_result_msg = std::make_unique<std_msgs::msg::Bool>();
      test_result_msg->data = false;
      test_result_publisher_->publish(std::move(test_result_msg));

      rclcpp::shutdown();
    }
  }

  void check_test_passed()
  {
    if (current_odometry_ >= TEST_PASS_ODOMETRY_DISTANCE) {
      calculate_score();

      RCLCPP_INFO(this->get_logger(), "Test passed! Score: %.2f", current_score_);

      // Communicating to other nodes the result of the test
      auto test_result_msg = std::make_unique<std_msgs::msg::Bool>();
      test_result_msg->data = true;
      test_result_publisher_->publish(std::move(test_result_msg));

      rclcpp::shutdown();
      }
  }

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr odometry_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr test_result_publisher_;

private:
	float current_odometry_;
	float current_minimum_distance_;
	unsigned int current_test_time_;
	unsigned long start_time_;
	float current_score_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestScore>());
  rclcpp::shutdown();
  return 0;
}