#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <string>
#include <unistd.h>
#include <time.h>
#include <math.h>

using std::placeholders::_1;

class Classifier : public rclcpp::Node
{
public:
	Classifier() : Node("classification"
)	{
		// Subscribe to clustered data 
  		clustered_data_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>("/clusters", 
			 rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
			std::bind(&Classifier::classifier_callback, this, _1));

  		// Advertise classication data
    	class_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/classified_clusters", rclcpp::SystemDefaultsQoS());		
	}

private:

	void classifier_callback(const std_msgs::msg::Float32MultiArray::SharedPtr cluster_msg)
	{
		uint rows = cluster_msg->layout.dim[0].size;
		// uint cols = cluster_msg->layout.dim[1].size;

		auto classification_msg = std_msgs::msg::Float32MultiArray();
		
		for(uint i=0; i<rows; i++){

			float x = cluster_msg->data[i*cluster_msg->layout.dim[1].stride + 0];
			float y = cluster_msg->data[i*cluster_msg->layout.dim[1].stride + 1];
			float distance = sqrt(pow(x,2) + pow(y,2));

			//Calculate pedestrian threshold curve
			float pedestrian_threshold = -16.9088 * log(0.118088 * distance);
			if (pedestrian_threshold < 4)
				pedestrian_threshold = 4;

			//Calculate vehicle threshold curve
			float vehicle_threshold = -60.5402 * log(0.0999325 * distance);
			if (vehicle_threshold < 8)
				vehicle_threshold = 8;

			//Calculate the distances between the two thresholds
			int cluster_size = cluster_msg->data[i*cluster_msg->layout.dim[1].stride + 3];
			float pedestrian_threshold_distance = abs(cluster_size - pedestrian_threshold);
			float vehicle_threshold_distance = abs(cluster_size - vehicle_threshold);

			//Push back centroid and distance data to the message
			classification_msg.data.push_back(x);
			classification_msg.data.push_back(y);
			classification_msg.data.push_back(distance);

			//Clasify obstacle depending on the distance to the closest threshold
			if(pedestrian_threshold_distance <= vehicle_threshold_distance){
				printf("Cluster %d with distance %f is a person! \n", i, distance);
				classification_msg.data.push_back(0);
			}else{
				printf("Cluster %d with distance %f is a car! \n", i, distance);
				classification_msg.data.push_back(1);
			}
		}
		printf("\n\n");

		//Construct classification_msg layout as unfolded two dimensional array

	    //Each row represents a cluster while each column contains data of x centroid, 
	    //y centroid, distance from vehicle and class of obstacle (0 for person, 1 for car) in this order
	    classification_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	    classification_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	    classification_msg.layout.dim[0].label = "clusters";
	    classification_msg.layout.dim[0].size = rows;
	    classification_msg.layout.dim[0].stride = 4*rows;
	    classification_msg.layout.dim[1].label = "classification_data";
	    classification_msg.layout.dim[1].size = 4;
	    classification_msg.layout.dim[1].stride = 4;
	    classification_msg.layout.data_offset = 0;

		//Publish
		class_pub_->publish(classification_msg);
	}
	
  	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr class_pub_;
  	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr clustered_data_subscription;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Classifier>());
	rclcpp::shutdown();

	return 0;
}