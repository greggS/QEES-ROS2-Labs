# QEES-ROS2-Labs
Prototypes of ROS2 based assignments prepared for the Quantitative Evaluation of Embedded Systems course at TUDelft

## Prerequirements
- Ubuntu (18.04 or newer)
- ROS2
- OpenGL support (most integrated chipsets)

## Overview
A simple ROS model of a car and a few sample scenarios are provided as the code part of the assignment. To better understand the functionality of each node, you can check the ***src*** folder which contains the source code for each node as well as a ***package.xml*** file that briefly describes the node functionality. Please have in mind, that the ***Markov*** node is just a skeleton that is supposed to be completed as a part of the assignment. The ***Collision Avoidance*** and ***Car Controller*** are also skeletons which will be part of the optional questions later on. 
![System schematic](/readme_images/architecture_schematic.png "System schematic")

The provided packages allow you to identify other objects (cars or pedestrians) via a Lidar sensor (mounted on the front bumper of the car) as well as determine their position and distance in relation to the car (see the .world file to see the sensor implementation). The clustering node calculates the relative distance from the front of the car to the center of an object cluster. 
![RViz example](/readme_images/rviz_tutorial.png "RViz example")

Based on the clustering node output, the classification node determines the type of object represented by the cluster. A sample where you see 4 cars and 1 person with relative distance 8.01, 13.6, 3.75, 10.6 and 3.66 from the center of the car is presented below.
![Gazebo example](/readme_images/Gazebo_tutorial.png "Gazebo example")

Note that the data you receive on topic clusters are relative distances and hence change due to the movement of the car. In order to get the absolute position values of an object, you need to use car's odometry information and reduce the movement of the car from the object's location. 

The classified clusters data is later passed to the Markov node which, based on a pedestrian behaviour model, calculates the  probabilities for the next pedestrian position. 
The prediction result is passed to the Collision Avoidance node, which should estimate the chance of collision based on the current velocity. 
The collision probability information is passed to the Car Control node, which merges it with the desired speed broadcasted by the Test Control node (which can be also considered the human driver). 
At the end, the Car Control node communicates the final desired speed to the simulation environment. 
A sample output from the clustering and classification nodes should look like this:
![Output example](/readme_images/output_example.png "Output example")

## Getting started
Here is list of simple instructions that will allow you to test your solutions and make sure you are able to analyse the provided data.

### Compiling the workspace: 
Once in the lab3_ws directory, execute the following commands:
$ colcon build --symlink-install
$ source install/setup.bash 
The final output should contain the following information: Summary: 7 packages finished.
   
### Running each of the provided nodes (and testing your modifications): 
Once you compiled the workspace, you can execute the following command to run, for example, the clustering node:
$ ros2 run clustering clustering
    
### Opening rviz2:
Execute the following command:
$ ros2 run rviz2 rviz2
Once Rviz opens, use the Add button on the bottom left of the window to add the /republished/ray/pointcloud and /republished/ray/laserscan topics. Make sure that for both topics. the unreliable parameter is checked. Also, you may have to change the top fixed_frame parameter to ray_link. Additionally, it might be helpful to increase the size of the points for better visualization. When closing Rviz, it is recommended that you save your configuration when prompted so that you don't have to repeat the procedure every time.
    
### Replying recorded data (rosbags):
Open the package with recorded data. Now, you can see each of the recorded scenarios using the following command (replace ROSBAG_NAME with the bag that you want to play):
$ ros2 bag play ROSBAG_NAME
Note that ROSBAG\_NAME represents the name of the folder containing all rosbag data, not any particular file. While the bag is replayed. Data is being published to all relevant topics of the simulation. You can use the 
$ ros2 topic list
to view which topics are available as well as 
$ ros2 topic echo TOPIC_NAME 
to view the data in real time. Furthermore, you can open Rviz to visualize what the sensor were seeing at the time. You can also run the clustering or classification nodes to see the logs. 
    
### Visualising the world in Gazebo:
To get an even better understanding of the provided pedestrian behavior scenarios, each scenarios has a generatedWorldWithActor.world file assigned to it, that you can visualise in Gazebo.
To open a world file in Gazebo just run the following command:
$ gazebo --verbose generatedWorldWithActor.world

### Analyzing the provided pedestrian scenarios
In order to create your pedestrian model, you need to analyse the data about the steps taken by the pedestrian in each of the provided scenarios. Each scenario folder also contains a generatedActor.csv file which contains raw data about walk waypoints. Scenarios from 10 to 15 contain the ground truth information about the states the pedestrian was in, in each specific point in time. You are free to choose the way you want to analyse the .csv files. 

## Sample test procedure
The following set of steps is an example of how you can approach running the provided test scenarios and testing your modifications:
In the Terminal, go to the laboratory workspace.
1. Compile the workspace:
$ colcon build --symlink-install
$ source install/setup.bash 
2. Open Rviz to visualize the sensor data (in a separate Terminal):
$ ros2 run rviz2 rviz2
3. (for scenarios 9 to 15) Run the classification and clustering nodes (run those commands in two separate terminals):
$ ros2 run clustering clustering
$ ros2 run classification classification
4. Navigate to a scenario directory
$ cd Scenarios/scenario_1
5. Start replaying a sample scenario:
$ ros2 bag play rosbag2_scenario_1
6. Run the Markov node:
$ ros2 run markov markov
7. Introduce changes to the Markov node in the markov.cpp file. 
8. You can return to step 1 