#! /usr/bin/env python

## @package bug_as  
# 
# @brief This is the bug_as node 
# @author Farouk Gasmi farouk.lionel100@gmail.com
# @version 1.0
# @date 25/05/2023
#  
#
# Description:  
#
# This is a Python script that implements the bug0 algorithm for navigation.
# This code implements the bug0 algorithm for navigation using ROS (Robot Operating System) and the Python programming language.
# The algorithm uses laser scan data and odometry information to navigate a robot to a desired position.
# @see action_user.py
# @see print_dis_avgvel.py
# @see goal_service.py
# @see go_to_point_service.py
# @see wall_follow_service.py

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from tf import transformations
from std_srvs.srv import *
import time

## Class for handling the bug0 algorithm.
#
# The code defines a class Bug0Algorithm that encapsulates the bug0 algorithm. 
# It has various member variables to store the robot's state, desired position, laser scan data, and ROS service clients.
# It also includes callback functions for the laser scan and odometry topics to update the relevant data.
class Bug0Algorithm:
    def __init__(self):
        # Initialize variables
        self.srv_client_go_to_point_ = None
        self.srv_client_wall_follower_ = None
        self.yaw_ = 0
        self.yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
        self.position_ = Point()
        self.pose_ = Pose()
        self.desired_position_ = Point()
        self.desired_position_.z = 0
        self.regions_ = None
        self.state_desc_ = ['Go to point', 'wall following', 'done']
        self.state_ = 0
        # 0 - go to point
        # 1 - wall following
        # 2 - done
        # 3 - canceled

        # Initialize ROS node
        rospy.init_node('bug0')

        # Subscribe to topics
        sub_laser = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)

        # Advertise topics
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Create service proxies
        self.srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
        self.srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)

        # Create the action server
        self.act_s = actionlib.SimpleActionServer('/reaching_goal', assignment_2_2022.msg.PlanningAction, self.planning, auto_start=False)
        self.act_s.start()

    ## Callback function for the laser scan topic subscriber.
    #
    # This function is called whenever a new laser scan message is received.
    # It updates the regions_ variable with the latest distance readings from the laser scanner.
    # @param msg The received LaserScan message.
    def clbk_laser(self, msg):
        self.regions_ = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }

    ## Callback function for the odometry topic subscriber.
    #
    # This function is called whenever a new odometry message is received.
    # It updates the position_, yaw_, and pose_ variables with the latest position and orientation information.
    # @param msg The received Odometry message.
    def clbk_odom(self, msg):
        # position
        self.position_ = msg.pose.pose.position
        self.pose_ = msg.pose.pose

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    ## Change the state of the algorithm.
    #
    # This function changes the state of the algorithm and performs the necessary actions accordingly.
    # which basically means that this function changes the state of the algorithm based on the desired state and performs the necessary actions.
    # The normalize_angle function normalizes an angle to the range [-pi, pi].
    # @param state The new state.
    def change_state(self, state):
        self.state_ = state
        log = "state changed: %s" % self.state_desc_[state]
        rospy.loginfo(log)
        if self.state_ == 0:
            resp = self.srv_client_go_to_point_(True)
            resp = self.srv_client_wall_follower_(False)
        elif self.state_ == 1:
            resp = self.srv_client_go_to_point_(False)
            resp = self.srv_client_wall_follower_(True)
        elif self.state_ == 2:
            resp = self.srv_client_go_to_point_(False)
            resp = self.srv_client_wall_follower_(False)

    ## Normalize an angle to the range [-pi, pi].
    #
    # This function normalizes an angle to the range [-pi, pi].
    # @param angle The angle to normalize.
    # @return The normalized angle.
    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    ## Execute the bug0 algorithm for navigation.
    #
    # This function is the main function that executes the bug0 algorithm for navigation based on the received goal.
    # It continuously checks the robot's position and orientation, updates the state, and publishes control commands.
    # @param goal The received goal.
    def planning(self, goal):
        change_state(0)
        rate = rospy.Rate(20)
        success = True

        self.desired_position_.x = goal.target_pose.pose.position.x
        self.desired_position_.y = goal.target_pose.pose.position.y
        rospy.set_param('des_pos_x', self.desired_position_.x)
        rospy.set_param('des_pos_y', self.desired_position_.y)

        feedback = assignment_2_2022.msg.PlanningFeedback()
        result = assignment_2_2022.msg.PlanningResult()

        while not rospy.is_shutdown():
            err_pos = math.sqrt(pow(self.desired_position_.y - self.position_.y, 2) +
                                pow(self.desired_position_.x - self.position_.x, 2))
            if self.act_s.is_preempt_requested():
                rospy.loginfo("Goal was preempted")
                feedback.stat = "Target cancelled!"
                feedback.actual_pose = self.pose_
                self.act_s.publish_feedback(feedback)
                self.act_s.set_preempted()
                success = False
                change_state(2)
                self.done()
                break
            elif err_pos < 0.5:
                change_state(2)
                feedback.stat = "Target reached!"
                feedback.actual_pose = self.pose_
                self.act_s.publish_feedback(feedback)
                self.done()
                break
            elif self.regions_ is None:
                continue
            elif self.state_ == 0:
                feedback.stat = "State 0: go to point"
                feedback.actual_pose = self.pose_
                self.act_s.publish_feedback(feedback)
                if self.regions_['front'] < 0.2:
                    change_state(1)
            elif self.state_ == 1:
                feedback.stat = "State 1: avoid obstacle"
                feedback.actual_pose = self.pose_
                self.act_s.publish_feedback(feedback)
                desired_yaw = math.atan2(
                    self.desired_position_.y - self.position_.y, self.desired_position_.x - self.position_.x)
                err_yaw = self.normalize_angle(desired_yaw - self.yaw_)
                if self.regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                    change_state(0)
            elif self.state_ == 2:
                break
            else:
                rospy.logerr('Unknown state!')

            rate.sleep()

        if success:
            rospy.loginfo('Goal: Succeeded!')
            self.act_s.set_succeeded(result)

    ## Publish a zero velocity Twist message.
    #
    # This function publishes a zero velocity Twist message to stop the robot.
    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.pub.publish(twist_msg)

    ## Main function for executing the bug0 algorithm.
    #
    # The main function initializes the desired position, creates the necessary subscribers, publishers, and service clients, and starts the bug0 algorithm.
    def main(self):
        time.sleep(2)

        # Initialize desired position
        self.desired_position_.x = 0.0
        self.desired_position_.y = 1.0
        rospy.set_param('des_pos_x', self.desired_position_.x)
        rospy.set_param('des_pos_y', self.desired_position_.y)

        # Initialize going to the point
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    bug0 = Bug0Algorithm()
    bug0.main()

