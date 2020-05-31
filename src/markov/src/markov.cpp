#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <string>
#include <unistd.h>
#include <time.h>
#include <math.h>

using std::placeholders::_1;

class Markov : public rclcpp::Node
{
public:
	Markov() : Node("markov"
)	{
		// Subscribe to classification data 
  		class_data_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>("/classified_clusters", 
			 rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
			std::bind(&Markov::markov_callback, this, _1));

  		// Advertise probability data
    	markov_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/markov_prob", rclcpp::SystemDefaultsQoS());		
	}

private:

	void markov_callback(const std_msgs::msg::Float32MultiArray::SharedPtr classification_msg)
	{
		uint rows = classification_msg->layout.dim[0].size;
		uint cols = classification_msg->layout.dim[1].size;
		float data_array[rows][cols];

		//Construct output message
		auto markov_output_msg = std_msgs::msg::Float32MultiArray();

		//------ WRITE YOUR CODE HERE -------//

		//Demo on how to parse the classifier message and print it in the terminal
		//The provided message as well as the output message are in the form of unfolded 2D arrays
		//In this case msg[i][j] = msg[i*dimension_1_stride + j] as showcased below
		//The data_array is only used in this context to showcase the example and not otherwise useful

 		printf(" x     y     dist   class \n"
	          "---------------------------\n");
		for (uint i=0; i<rows; i++){
			for (uint j=0; j<cols; j++){
				data_array[i][j] = classification_msg->data[i*classification_msg->layout.dim[1].stride + j];
				printf("%5.2lf ", data_array[i][j]);
			}
			printf("\n");
		}
		printf("\n\n");

		//On the data_array each row represents a cluster of points, while each column contains
		//the centroid of the x coordinates of the cluster in position 0, the centroid of the y
		//coordinates on position 1, the distance of the cluster from the current position of the 
		//vehicle on position 2 and the class of the object (0 for a pedestrian, 1 for car) on position 3

		//Can differ from the number of objects as each located obstacle can be on multiple possitions 
		//with a different probability
		uint number_of_object_probabilities = rows;

		//Some code based on the provided info
		for (uint i=0; i<number_of_object_probabilities; i++){
			
			float probability = 1.0;
			float x = data_array[i][0];
			float y = data_array[i][1];
			float distance = data_array[i][2];
			float obj_class = data_array[i][3];
	
			//Assumes that the object will stay in its current position with 100% probability

			//Push the computed data in the message (object has X probability of being in position x,y in the next time step)
			markov_output_msg.data.push_back(x);
			markov_output_msg.data.push_back(y);
			markov_output_msg.data.push_back(probability);
		}


		// -------- TO HERE ---------//

		//Construct markov_output_msg layout as an unfolded two dimensional array

	    //Each row represents a cluster while each column contains data of x coordinate, 
	    //y coordinate and the probability that the detected object will be in that position
	    markov_output_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	    markov_output_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	    markov_output_msg.layout.dim[0].label = "objects";
	    markov_output_msg.layout.dim[0].size = number_of_object_probabilities; 	//This value should depend on the number of position probabilities
	    markov_output_msg.layout.dim[0].stride = 3*number_of_object_probabilities;
	    markov_output_msg.layout.dim[1].label = "probability_data";
	    markov_output_msg.layout.dim[1].size = 3;
	    markov_output_msg.layout.dim[1].stride = 3;
	    markov_output_msg.layout.data_offset = 0;

		//publish probabilities
		markov_pub->publish(markov_output_msg);

	}
	
  	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr markov_pub;
  	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr class_data_subscription;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Markov>());
	rclcpp::shutdown();

	return 0;
}