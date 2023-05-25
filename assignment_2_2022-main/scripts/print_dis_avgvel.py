#! /usr/bin/env python

## @package print_dis_avgvel
#
# @brief This is the print_dis_avgvel node 
# @author Farouk Gasmi farouk.lionel100@gmail.com
# @version 1.0
# @date 25/05/2023
#
# Description:
#
# The third node prints out information about a robot's distace from target and average velocity.
# The node gets the publish frequency parameter from ROS parameters, 
# which is used to determine how often the information is printed. 
# It also initializes a variable to keep track of the last time the information was 
# printed and creates a subscriber to the '/posxy_velxy' topic, which i to containining
# messages of robot's curren x,y positions and x,y velocities.
#
# @see bug_as.py
# @see action_user.py
# @see goal_service.py
# @see go_to_point_service.py
# @see wall_follow_service.py

import rospy
import math
import time
from assignment_2_2022.msg import Posxy_velxy
from colorama import init
init()
from colorama import Fore, Back, Style

## Class for printing distance and average velocity information.
class PrintInfo:
    def __init__(self):
        # Get the publish frequency parameter
        self.freq = rospy.get_param("frequency")

        # Last time the info was printed
        self.printed = 0

        # Subscriber to the position and velocity topic
        self.sub_pos = rospy.Subscriber("/posxy_velxy", Posxy_velxy, self.posvel_callback)

    ## Callback function for the position and velocity subscriber.
    #
    # This function is called whenever a new position and velocity message is received.
    # It computes the distance to the target position and the average speed of the robot,
    # and prints the information if the elapsed time exceeds the publish frequency.
    # @param msg The received Posxy_velxy message.
    def posvel_callback(self, msg):
        # Compute time period in milliseconds
        period = int((1.0 / self.freq) * 1000)
        
        # Get current time in milliseconds
        curr_time = int(time.time() * 1000)

        # Check if the current time minus the last printed time is greater than the period
        if curr_time - self.printed > period:
            # Get the desired position from ROS parameters
            target_x = rospy.get_param("des_pos_x")
            target_y = rospy.get_param("des_pos_y")

            # Get the actual position of the robot from the message
            robot_x = msg.msg_pos_x
            robot_y = msg.msg_pos_y

            # Compute the distance between the desired and actual positions
            distance = round(math.dist([target_x, target_y], [robot_x, robot_y]), 2)
            
            # Get the actual velocity of the robot from the message
            vel_x = msg.msg_vel_x
            vel_y = msg.msg_vel_y           

            # Compute the average speed using the velocity components from the message
            average_speed = round(math.sqrt(vel_x ** 2 + vel_y ** 2), 2)

            # Print the distance and speed information
            rospy.loginfo(Fore.GREEN + "Distance to target: %s [m]", distance)
            rospy.loginfo(Fore.MAGENTA + "Robot average speed: %s [m/s]", average_speed)

            # Update the last printed time
            self.printed = curr_time

## Main function.
# This is the main function.
def main():
	
    # Suppress the timestamps from the log messages
    #rospy.set_param('/rosconsole/formatter/time', 'none') I tried to delete time values from screen but it didn't work	
    
    # Initialize the node
    rospy.init_node('print_dis_avgvel')
    
    # Create an instance of the PrintInfo class
    print_dis_avgvel = PrintInfo()
    
    # Wait for messages
    rospy.spin()

if __name__ == "__main__":
    main()

