# Description of the program
A collection of tools and libraries are available under the open-source ROS framework, which is used to build and manage robotic systems. It includes a publisher and subscriber for sending and receiving data, an action client for asking a server for information and receiving the results, a launch file for starting and setting up nodes, a custom service for developing new request-response patterns, and a custom message for sending data between nodes. These capabilities allow for communication between various robotic system components, making it simple to develop, test, and deploy sophisticated robotic applications.

For this task, you must build a Gazebo and Rviz package for a robot simulation. This assignment's goal is to design a new ROS package, for which we will develop three nodes:
1. A node that implements an action client that enables the user to establish or cancel a target (x, y). By using the values published on the subject /odom, the node additionally broadcasts the robot's location and velocity as a custom message (x, y, vel x, vel y).
2. A service node that publishes the total number of objectives achieved and abandoned when it is called.
3. A node that prints the robot's average speed as well as its distance from the goal and subscribes to the robot's location and velocity using a custom message. To control how quickly the node broadcasts the data, use a parameter.
4. Make a **launch file** as well to kick off the entire simulation. Set the value for the **frequency** of the information published by node 3.

## Instructions for running the project
### Starting the simulation
you can start the simulation by following these steps
* 1- Make sur to be in the source file of your workspace /work_space/src
* 2- clone the repository <git clone https://github.com/Faroukgsmi26/Assignment_number_2 >
* 3- build the workspace by <catkin_make>
* 4- launch the launch file **solution.launch** by <roslaunch solution.launch> 
     by launching the file, the follwing nodes starts automatically 
     * rviz
     * Gazebo
     * user_interface.cpp
     * robot_controller.cpp 
     * teleop_twist_keyboard.py 
 
 ### Starting jupyter notebook 
 you can start the jupyter notebook by 
* 1- make sure you are in /work_space/Assignment2
* 2- excute the command <juputer notebook --allow-root>
  

## Documentation: Doxygen
The architecture of the assignment is documented by using Doxygen, you can access the documentations by this link 
<https://faroukgsmi26.github.io/Assignment_number_2/>

## Jupyter Notebook 
### Definition
The Jupyter Notebook is an open source web application that you can use to create and share documents that contain live code,
equations, visualizations, and text.


---------------------------------
## Node 1: Action (action_user.py)
The first node of our package creates a publisher "pub" that publishes a custom message "**Posxy_velxy**" on the topic "/posxy_velxy". The custom message contains four fields "**msg_pos_x**", "**msg_pos_y**", "**msg_vel_x**", "**msg_vel_y**" that represents the position and velocity of the robot.

A subscriber named "sub from Odom" is also created by the node, and it subscribes to the topic "/odom," which broadcasts the Odometry message. Every time a message is received on the subject "/odom," the callback function "publisher" is called. This function generates an instance of the custom message and extracts the location and velocity information from the odometry message. The function then publishes the message on the topic "/posxy velxy" after assigning the location and velocity data to the respective fields of the custom message.

After creating an action client, the "action client()" function waits for the action server "/reaching goal" to begin. It then starts a while loop, asking the user to input the goal's target location or to press "c" to abandon it. The action client cancels the goal and changes the status goal to false if the user types "c." When a target location is entered by the user, the function changes the inputs from strings to floats, produces a goal, and then transmits the goal to the action server (Planning.action). Moreover, status goal is set to true.
A goal is sent to the action server using a straightforward action client implementation, and the goal's outcome, which may be an error, a success, or a cancelation, is then awaited.

--------------------------------------------------------------------------------------------------------------------------------------------------
## Node 2: Service (goal_service.py)
The second node provides a ROS service that answers with the total number of objectives achieved and abandoned in response to queries on the "goal service" topic. Moreover, it refreshes the counters for goals that have been attained and cancelled in accordance by subscribing to the topic "/reaching goal/result" and receiving messages on goal status. The current values of goal reached and goal cancelled are returned in a goal rcResponse message when the service is invoked.

It generates an instance of the Service class and initializes a ROS node named "goal service." This establishes the service, which is a subscriber to the "/reaching goal/result" topic and listens for requests on the "goal service" topic. The data method is called in response to a request on the "goal service" topic, and it returns a goal rcResponse message with the current values of goal reached and goal cancelled.

The result callback method is invoked whenever a message on the "/reaching goal/result" topic is received. This method looks at the message's goal's status (while the robot is moving, status is 1, when the target is cancelled, status is 2, and when the robot reaches the target, status is 3), and it increases the relevant counter, either goal cancelled or goal reached. Run "rostopic echo /reaching goal/status" to see the current situation.

--------------------------------------------------------------------------------------------------------------------------------------------------
## Node 3: Print Distance and Average Velocity (print_dis_avgvel.py)
The third node puts out data on a robot's average velocity and distance from the target. The publish frequency parameter, which controls how frequently the information is printed, is obtained by the node from ROS parameters. Moreover, it sets up a variable to record when the data was last printed and adds a subscriber to the '/posxy velxy' topic, which will receive messages containing the robot's current x, y coordinates and x, y velocities.

The node initially obtains the desired and actual positions of the robot from the message received. It then uses the math.dist() method to determine the distance between the intended and actual places. It also obtains the robot's real velocity from the message and computes the average speed using the velocity components from the message. Lastly, it uses the rospy.loginfo() method to display the distance and average speed statistics, as well as to update the last reported time variable.

-------------------------------------
## Creating Launch file (assignment2.launch)
The ROS launch file is used to simultaneously start numerous nodes and adjust settings. The launch file is written in XML and utilizes the launch> tag as its root element. The launch file begins by incorporating another launch file, "sim w1.launch," which is already included in our package and is used to run Gazebo and Rviz simulators as well as environment-related nodes. It then sets the parameters "des pos x" and "des pos y" to 0.0 and 1.0, respectively. These characteristics are utilized by other nodes to establish the robot's target location.

Next we set the parameter "frequency" to 1.0. The node "print dis avgvel.py" uses this parameter to decide how frequently the distance and average velocity information should be printed.

After that, it starts nodes using the <node> tag, these nodes are:

  +  "wall_follower.py"
  +  "go_to_point.py"
  +  "bug_action_service.py"
  +  "action_user.py"
  +  "goal_service.py"
  +  "print_dis_avgvel.py"

Each of these nodes is specified by stating the package name "assignment 2 2022" in which it resides, the file type, and the node name. The final two nodes are launched with the extra parameters output="screen" and launch-prefix="xterm -hold -e," which force the output of these nodes to be printed to the screen in a new terminal window.
```xml
<?xml version="1.0"?>
<launch>
    <include file="$(find assignment_2_2022)/launch/sim_w1.launch" />
    <param name="des_pos_x" value= "0.0" />
    <param name="des_pos_y" value= "1.0" />
    
    <!--Frequency parameter to set the frequency of the print_dis_avgvel node -->
    <param name="frequency" type="double" value="1.0" />
    
    <node pkg="assignment_2_2022" type="wall_follow_service.py" name="wall_follower" />
    <node pkg="assignment_2_2022" type="go_to_point_service.py" name="go_to_point"  />
    <node pkg="assignment_2_2022" type="bug_as.py" name="bug_action_service" output="screen" />
    <node pkg="assignment_2_2022" type="action_user.py" name="action_user" output="screen" launch-prefix="xterm -hold -e" />
    <node pkg="assignment_2_2022" type="goal_service.py" name="goal_service"  />
    <node pkg="assignment_2_2022" type="print_dis_avgvel.py" name="print_dis_avgvel" output="screen" launch-prefix="xterm -hold -e" />
</launch>
```
Instead of starting each node and setting each parameter separately, this launch file allows you to start all of the application's nodes and specify the relevant parameters with a single command. It also enables the nodes to be operated in a certain order and with precise parameters.

------------------------------------
## Installation
First of all before running the program it is required to install the xterm libray. Open a terminal window and run the following command to install the xterm package, this library helps us to print outputs of the nodes in a new terminal window :

```command
	sudo apt-get install xterm -y
```
Next, navigate to your ROS workspace 'src' folder and clone this repository using the following command:
	
```command
	git clone <link of the repository>
```
Once the repository has been cloned, navigate to the work space drectory and run the following command to build the package:

```command
	catkin_make
```
After the package has been built successfully, finally, we can launch the simulation.

---------------------------------

## How To Run The Simulation
The assignment's launch file may be located in the "launch" folder under the "assignment 2 2022" directory. Use the following command to begin the simulation:
```command
	roslaunch assignment_2_2022 assignment2.launch
```
After the robot is successfully launched, four screens should appear: one for entering target coordinates (action user.py), one for displaying the robot's distance and average velocity (print dis avgvel.py), and two for the Gazebo and Rviz visualization environments.

To run the service node (goal_service.py) that, when called, prints the number of goals reached and cancelled use the following command:

```command
	rosservice call /gaol_service
```
or you can just use the rqt utility to invoke the service. rqt is a ROS (Robot Operating System) application that provides a simple and intuitive GUI (graphical user interface) for troubleshooting and analyzing system components. It enables users to monitor several data streams, such as topics, services, and parameters, as well as conduct operations like graphing, logging, and debugging. rqt also has a plugin system that enables developers to design custom plugins for certain purposes. It aids in the debugging, visualization, and inspection of the ROS system, as well as the monitoring and control of the ROS nodes and subjects.

```command
	rqt
```

To access the Service Caller function in rqt, follow these steps:

    1- Go to the "Plugins" menu
    2- Select "Services"
    3- Click on "Service Caller"
    4- In the Service Caller window, locate the "goal_service"
    5- Click the "Call" button
    6- The response window will display the number of targets reached and cancelled.
    
-----------------------------------
		
## Conclusion
In this assignment, three nodes were created: (a) an action client node that allows the user to set or cancel a target (x, y) and also publishes the robot's position and velocity as a custom message; (b) a service node that prints the number of goals reached and cancelled; and (c) a node that subscribes to the robot's position and velocity and prints the robot's distance from the target and average speed. A launch file was also generated to begin the simulation and to specify how frequently node (c) publishes information. Overall, the package shows how to utilize action clients, services, and custom messages in ROS to operate a robot and track its performance. Some suggestions for improving this assignment include:

1-Utilizing markers in RViz to more intuitively communicate the desired location and the robot's current position, for as by using various colors or shapes to signify different states (e.g. goal reached, goal canceled).

2-Integrating the robot's orientation into the display, for example, by using an arrow or a 3D representation of the robot to show which way it is facing.

3-Using Gazebo's built-in visualization features, such as placing a marker or a flag at the target point, to display the target position in the simulated world.

4-Integrating a route-planning algorithm to display the robot's intended path to the destination, such as by utilizing RViz's "Path" display type.

5-Input validation is used to guarantee that users may only submit integers and not other sorts of input such as floating point numbers or characters. Whenever an incorrect input is entered, provide feedback to the user by showing an error message or marking the input field in red.

6-Including a check to see if the input value is within a specific range, and if not, prompting the user to enter a value that is inside the range.
