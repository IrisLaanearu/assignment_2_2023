#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import PlanningActionGoal, RobotState
from geometry_msgs.msg import Point, Pose, Twist
from std_srvs.srv import *
import math

# Global variables
# Goal initialization
goal = Point()
goal.x = rospy.get_param('des_pos_x')
goal.y = rospy.get_param('des_pos_y')
goal.z = 0
# Current position initialization
position = RobotState()
# Sevice active state
active = False
# Velocity list to hold previous data
velocities = []

# Get the averaging window size parameter from the launch file
avg_window_size = rospy.get_param('avg_window_size')

# Callbacks
# Callback from the RobotState topic
def state_callback(msg):
    global position, velocities, avg_window_size
    # Extract position and velocity information from the RobotState message
    position.x = msg.x
    position.y = msg.y
    position.vel_x = msg.vel_x
    position.vel_z = msg.vel_z
    
    # Save the velocity to the list
    velocities.append(position.vel_x)
    # Keep only the last samples of the robot's velocity
    if len(velocities) > avg_window_size:
    	velocities.pop(0) # Remove the oldest velocity in the list

# Callback from the reaching_goal/goal topic    
def goal_callback(msg):
    global goal
    goal.x = msg.goal.target_pose.pose.position.x
    goal.y = msg.goal.target_pose.pose.position.y     
    
# Service callback    
def distance_from_goal_handler(req):
    global goal, position, velocities, active
    
    # Declare distance as a local variable
    distance = Point()
    
    # Calculate distance from the goal
    distance.x = goal.x - position.x
    distance.y = goal.y - position.y
    # Print the distance from the goal
    print(f"\nDistance from the goal: x = {distance.x:.4f}, y = {distance.y:.4f}")
    
    # Calculate the average speed
    # Check if there are values in the velocities list
    if velocities:
        avg_speed = sum(map(abs, velocities)) / len(velocities)
    else: 
        avg_speed = 0.0		
    # Print the average speed
    print(f"Average speed: v = {avg_speed:.4f}")

    # Return the status of the service call
    active = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = 'Done!'
    return response    

def main():
    # Initialize the node	
    rospy.init_node('distance_from_goal')
    
    # Subscribe to the RobotState topic
    sub_odom = rospy.Subscriber('/RobotState', RobotState, state_callback)
    
    # Subscribe to the reaching_goal/goal topic
    sub_goal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)
    
    # Initialize the service
    srv = rospy.Service('distance_from_goal', SetBool, distance_from_goal_handler)
    
    rospy.spin()

if __name__ == '__main__':
    main()
