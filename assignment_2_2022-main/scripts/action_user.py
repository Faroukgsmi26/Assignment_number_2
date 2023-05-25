#!/usr/bin/env python

## @package action_user
# @brief This the action_user node!
# @author Farouk Gasmi farouk.lionel100@gmail.com
# @version 1.0
# @date 25/05/2023
#
# Description:
#
# This is a Python script is a node that implements an action client, allowing the user to set a target (x, y) or to cancel it. 
# The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_y), by relying on the values published on the topic /odom.
# @see bug_as.py
# @see print_dis_avgvel.py
# @see goal_service.py
# @see go_to_point_service.py
# @see wall_follow_service.py

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
import sys
import select
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2022.msg import Posxy_velxy
from colorama import Fore, Style
from colorama import init

init()

## Callback function for the subscriber.
#
# The first node of our package creates a publisher "pub" that publishes a custom message "Posxy_velxy" on the topic "/posxy_velxy".
# The custom message contains four fields "msg_pos_x", "msg_pos_y", "msg_vel_x", "msg_vel_y",
# that represent the position and velocity of the robot.
# The callback function "publisher" is called every time a message is received on the topic "/odom".
def publisher(msg):
    global pub
    # Get the position information
    pos = msg.pose.pose.position
    # Get the velocity information
    velocity = msg.twist.twist.linear
    # Custom message
    posxy_velxy = Posxy_velxy()
    # Assign the parameters of the custom message
    posxy_velxy.msg_pos_x = pos.x
    posxy_velxy.msg_pos_y = pos.y
    posxy_velxy.msg_vel_x = velocity.x
    posxy_velxy.msg_vel_y = velocity.y
    # Publish the custom message
    pub.publish(posxy_velxy)

## Action client function.
#
# The "action_client()" funtion creates an action client and waits for the action server "/reaching_goal" to start.
# It enters a while loop that prompts the user to enter the target position or type "c" to cancel the goal. If the user enters "c", the action client cancels the goal and sets the status_goal to false.
# If the user inputs a target position, the function converts the inputs from strings to floats, creates a goal with the target position and sends it to the action server(Planning.action).
# It also sets status_goal to true. It's a simple implementation of action client, it sends a goal to the action server and waits for the result of the goal,
# it could be an error, a success, or a cancelation. The user can interact with the client, setting a goal or canceling it.
def action_client():
    """! Acts like the main function, allowing the user to access and modify the ontology and change the map_state.
    @param No parameters
    @return No returned value
    """
    # Create the action client
    action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    # Wait for the server to be started
    action_client.wait_for_server()
    
    status_goal = False
	
    while not rospy.is_shutdown():
        # Get the keyboard inputs
        print(Fore.GREEN + "Please enter the position of the target or type 'c' to cancel it.")
        x_pos_input = input(Fore.MAGENTA + "X position of target: ")
        y_pos_input = input(Fore.MAGENTA + "Y position of target: ")
        
        # If the user entered 'c', cancel the goal
        if x_pos_input == "c" or y_pos_input == "c":
            # Cancel the goal
            action_client.cancel_goal()
            status_goal = False
        else:
            # Convert numbers from string to float
            x_pos_send = float(x_pos_input)
            y_pos_send = float(y_pos_input)
            # Create the goal to send to the server
            goal = assignment_2_2022.msg.PlanningGoal()
            goal.target_pose.pose.position.x = x_pos_send
            goal.target_pose.pose.position.y = y_pos_send
					
            # Send the goal to the action server
            action_client.send_goal(goal)
            status_goal = True

## Main function.
def main():
    # Initialize the node
    rospy.init_node('action_user')
    # Global publisher
    global pub
    # Publisher: Send a message which contains two parameters (velocity and position)
    pub = rospy.Publisher("/posxy_velxy", Posxy_velxy, queue_size=1)
    # Subscriber: Get from "Odom" two parameters (velocity and position)
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher)
    # Call the action_client function
    action_client()

if __name__ == '__main__':
    main()

