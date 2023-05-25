#! /usr/bin/env python

## @package wall_follower_service
# @brief This the wall_follower_service node!
# @author Farouk Gasmi farouk.lionel100@gmail.com
# @version 1.0
# @date 25/05/2023
#
# Description:
#
# This is a Python script that implements a wall follower behavior using laser scan data.
# The robot navigates in an environment with walls and adjusts its movement to follow the contours of the walls.
# @see bug_as.py
# @see print_dis_avgvel.py
# @see goal_service.py
# @see go_to_point_service.py
# @see action_user.py
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

## Global variable for the wall follower activation state.
active_ = False

## Global variable for the publisher to control the robot's movement.
pub_ = None

## Dictionary to store the laser scan regions.
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

## Global variable for the state of the wall follower.
state_ = 0

## Dictionary to map state numbers to state descriptions.
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

## Service callback function to switch the wall follower on or off.
# This function is a service callback that handles the switching on or off of the wall follower behavior.
# It takes a request message 'req' as input.
# It updates the global variable 'active_' based on the value of 'req.data'.
# It returns a response message indicating the success of the switch operation.
# @param req The request message.
def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

## Callback function for the laser scan subscriber.
# This is a callback function for laser scan subscriber.
# It receives a LaserScan message 'msg' as input.
# It extracts the ranges from the laser scan message and assigns them to the 'regions_' dictionary, representing different regions of the robot's surroundings.
# It then calls the 'take_action()' function to perform appropriate actions based on the current regions.
# @param msg The received LaserScan message.
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()

## Function to change the state of the wall follower.
# This function changes the state of the wall follower behavior.
# It takes the new state as input.
# It updates the global variable 'state_' with the new state.
# If the state has changed, it prints a message indicating the current state.
# @param state The new state.
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

## Function to take action based on the laser scan regions.
# This function determines the action to be taken based on the current regions of the laser scan.
# It calculates various distance thresholds ('d0' and 'd') to define different cases.
# Based on the values of the regions, it selects the appropriate case and changes the state using the 'change_state()' function.
# The function prints the state description for debugging purposes.
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

## Function to control the robot's movement to find the wall.
# This function generates a 'Twist' message to control the robot's movement to find the wall.
# It sets a linear velocity 'msg.linear.x' to 0.2 and an angular velocity 'msg.angular.z' to -0.3, causing the robot to move forward while slightly turning to the left.
# It returns the generated 'Twist' message.
# @return The 'Twist' message to be published.
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

## Function to control the robot's movement to turn left.
# This function generates a 'Twist' message to control the robot's movement to turn left.
# It sets an angular velocity 'msg.angular.z' to 0.3, causing the robot to rotate in place to the left.
# It returns the generated 'Twist' message.
# @return The 'Twist' message to be published.
def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

## Function to control the robot's movement to follow the wall.
# This function generates a 'Twist' message to control the robot's movement to follow the wall.
# It sets a linear velocity 'msg.linear.x' to 0.5, causing the robot to move forward at a moderate speed.
# It returns the generated 'Twist' message.
# @return The 'Twist' message to be published.
def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg

## Main function.
# This is the main function.
def main():
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()

