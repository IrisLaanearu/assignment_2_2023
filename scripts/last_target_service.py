#!/usr/bin/env python3

import rospy
from assignment_2_2023.msg import PlanningActionGoal
from geometry_msgs.msg import Point, Pose
from std_srvs.srv import *

# Global variables to store the last target coordinates
last_target = Point()
last_target.x = rospy.get_param('des_pos_x')
last_target.y = rospy.get_param('des_pos_y')
last_target.z = 0

active_ = False

# Service callback function
def last_target_handler(req):
    global last_target, active_
    # Print the last target coordinates to the console
    print(f"\nLast Target set by the user: x = {last_target.x:.4f}, y = {last_target.y:.4f}")


    # You can also return the coordinates as part of the service response
    active_ = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = 'Done!'
    return response

# Callback function for the reaching_goal/goal topic
def goal_callback(msg):
    global last_target
    last_target.x = msg.goal.target_pose.pose.position.x
    last_target.y = msg.goal.target_pose.pose.position.y 

def main():
    rospy.init_node('last_target')

    # Subscribe to the reaching_goal/goal topic
    sub_goal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)

    # Create a ROS service
    rospy.Service('last_target', SetBool, last_target_handler)

    rospy.spin()

if __name__ == '__main__':
    main()

