#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include <chrono>
#include <string>
#include <unistd.h>
#include <time.h>

#include "dbscan.h"

#define MINIMUM_POINTS 4     // minimum number of cluster
#define EPSILON (1*1)  // distance for clustering, metre^2

using std::placeholders::_1;

class Clustering : public rclcpp::Node
{
public:
	Clustering() : Node("clustering"
)	{
		// Subscribe to PointCloud
  		laser_data_subscription = this->create_subscription<sensor_msgs::msg::PointCloud>("/republish/ray/pointcloud", 
			 rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
			std::bind(&Clustering::processPointCloud, this, _1));

  		// Advertise cluster data
    	cluster_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/clusters", rclcpp::SystemDefaultsQoS());		
	}

private:

	void outputClusters(vector<Point>& points, int num_points)
	{
		Point *p = (Point *)calloc(num_points, sizeof(Point));

		int i = 0;
		int num_of_clusters = 0;
		while (i < num_points)
	    {
	    	p[points[i].clusterID-1].x += points[i].x;
	    	p[points[i].clusterID-1].y += points[i].y;
	    	p[points[i].clusterID-1].z += points[i].z;
	    	p[points[i].clusterID-1].clusterID ++;

	    	if(points[i].clusterID > num_of_clusters){
	    		num_of_clusters = points[i].clusterID;
	    	}

	        ++i;
	    }

	    //Output message containing number of clusters and centroid data
	    auto cluster_msg = std_msgs::msg::Float32MultiArray();
	    int outliers = 0;

		printf(" x     y     z     cluster_size\n"
	        "-----------------------------\n");
	    for(i=0; i<num_of_clusters; i++){
	    	
	    	if(p[i].clusterID < 4){
	    		outliers ++;
	    		continue;
	    	}

	    	p[i].x = p[i].x / p[i].clusterID;
	    	p[i].y = p[i].y / p[i].clusterID;
	    	p[i].z = p[i].z / p[i].clusterID;

	    	printf("%5.2lf %5.2lf %5.2lf: %d\n",
	                 p[i].x,
	                 p[i].y, p[i].z,
	                 p[i].clusterID);

	    	//Populate output message
	    	cluster_msg.data.push_back(p[i].x);
	    	cluster_msg.data.push_back(p[i].y);
	    	cluster_msg.data.push_back(p[i].z);
	    	cluster_msg.data.push_back(p[i].clusterID);
	    }

	    //Construct cluster_msg layout as unfolded two dimensional array

	    //Each row represents a cluster while each column contains data of x centroid, 
	    //y centroid, z centroid and number of points in the cluster in this order
	    cluster_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	    cluster_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	    cluster_msg.layout.dim[0].label = "clusters";
	    cluster_msg.layout.dim[0].size = num_of_clusters-outliers;
	    cluster_msg.layout.dim[0].stride = 4*(num_of_clusters-outliers);
	    cluster_msg.layout.dim[1].label = "cluster_data";
	    cluster_msg.layout.dim[1].size = 4;
	    cluster_msg.layout.dim[1].stride = 4;
	    cluster_msg.layout.data_offset = 0;

	    //Publish
		cluster_pub_->publish(cluster_msg);

	    free(p);
	}

	void processPointCloud(const sensor_msgs::msg::PointCloud::SharedPtr scan) 
	{
		// construct vector of point data
		vector<Point> points;

		for(uint i=0; i<scan->points.size(); i++){
			Point p;

			p.x = scan->points[i].x;
			p.y = scan->points[i].y;
			p.z = scan->points[i].z;

			p.clusterID = UNCLASSIFIED;
			points.push_back(p);
		}

		// constructor
	    DBSCAN ds(MINIMUM_POINTS, EPSILON, points);

    	// main loop
    	ds.run();

    	// result of DBSCAN algorithm
    	if (ds.getTotalPointSize() > MINIMUM_POINTS-1){
    		outputClusters(ds.m_points, ds.getTotalPointSize());	
    	}
	}

  	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cluster_pub_;
  	rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr laser_data_subscription;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Clustering>());
	rclcpp::shutdown();

	return 0;
}