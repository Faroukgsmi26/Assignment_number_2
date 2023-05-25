#! /usr/bin/env python

## @package goal_service
#
# @brief This is the goal_service node 
# @author Farouk Gasmi farouk.lionel100@gmail.com
# @version 1.0
# @date 25/05/2023
#
# Description: 
#
# This is a Python script that implements a service for tracking goals reached and cancelled.
# node creates a ROS service that listens for requests on the "goal_service" topic, 
# and responds with the number of goals reached and cancelled. 
# It also subscribes to the "/reaching_goal/result" topic to receive messages about the status of
# goals and updates the counters for goals reached and cancelled accordingly. 
# It initializes a ROS node called "goal_service" and creates an instance of the Service class.
#
# @see bug_as.py
# @see print_dis_avgvel.py
# @see action_user.py
# @see go_to_point_service.py
# @see wall_follow_service.py

import rospy  # Import the ROS python library
from assignment_2_2022.srv import goal_rc, goal_rcResponse  # Import the service and service response messages
import actionlib  # Import the actionlib library
import actionlib.msg  # Import the actionlib message library
import assignment_2_2022.msg  # Import the package message library

## Service class 
# This class is for tracking goals reached and cancelled.
class Service:
    def __init__(self):
        # Initialize the counters for goals reached and cancelled
        self.goal_cancelled = 0
        self.goal_reached = 0

        # Create the service
        self.srv = rospy.Service('goal_service', goal_rc, self.data)

        # Subscribe to the result topic
        self.sub_result = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult,
                                           self.result_callback)

    ## Callback function for the result topic subscriber.
    #
    # This function is called whenever a new result message is received.
    # It updates the counters for goals reached and cancelled based on the status of the result.
    # @param msg The received PlanningActionResult message.
    def result_callback(self, msg):
        # Get the status of the result from the msg
        status = msg.status.status

        # Goal cancelled (status = 2)
        if status == 2:
            self.goal_cancelled += 1

        # Goal reached (status = 3)
        elif status == 3:
            self.goal_reached += 1

    ## Service callback function.
    #
    # This function is called whenever a service request is received.
    # It returns a response containing the current values of goal_cancelled and goal_reached.
    # @param req The received goal_rc service request.
    # @return The goal_rc service response containing the current values of goal_cancelled and goal_reached.
    def data(self, req):
        # Return the response containing the current values of goal_cancelled and goal_reached
        return goal_rcResponse(self.goal_reached, self.goal_cancelled)


## Main function.
def main():
    # Initialize the node
    rospy.init_node('goal_service')

    # Create an instance of the Service class
    goal_service = Service()

    # Wait for messages
    rospy.spin()


if __name__ == "__main__":
    main()

