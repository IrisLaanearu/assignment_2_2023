#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import assignment_2_2023.msg
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from assignment_2_2023.msg import PlanningFeedback, RobotState

# Set the new goal function    
def set_goal(client, x, y):
    # Send a new goal to the action server
    goal = assignment_2_2023.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal)

# Cancel the goal function
def cancel_goal(client):
    # Check if the goal is active
    if client and client.get_state() == GoalStatus.ACTIVE:
        print("Cancelling goal...")
        client.cancel_goal()
        client.wait_for_result()  # Wait for the cancellation to be processed
        return True
    else:
        print("No active goal to cancel.")
        return False

# Callback function for the /odom topic 
def odom_callback(odom_msg, robot_state_publisher):
    # Extract position and velocity information from the odom message
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    vel_x = odom_msg.twist.twist.linear.x
    vel_z = odom_msg.twist.twist.angular.z

    # Publish the position and velocity on the RobotState topic
    robot_state_msg = RobotState(x=x, y=y, vel_x=vel_x, vel_z=vel_z) # Write the message
    robot_state_publisher.publish(robot_state_msg) # Publish the message       
        
def main():
    # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS
    rospy.init_node('user_interface')
    # Create the SimpleActionClient
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    # Wait until the action server has started up and started listening for goals
    client.wait_for_server()
    
    # Publisher to the RobotState custom message
    robot_state_publisher = rospy.Publisher('/RobotState', RobotState, queue_size=1)
    
    # Subscribe to the /odom topic
    odom_subscriber = rospy.Subscriber('/odom', Odometry, odom_callback, robot_state_publisher, queue_size=1)
    
    while not rospy.is_shutdown():
        rospy.sleep(1)  # Give some time so asking for goal would be the last
    	
        # User enters the goal coordinates
        x = float(input("Enter x coordinate for the goal: "))
        y = float(input("Enter y coordinate for the goal: "))

        # Send the goal
        set_goal(client, x, y)
        
        # Continuously check for status of the goal
        while not rospy.is_shutdown():
            rospy.sleep(1)  # Give some time for the goal to be processed
	    
	    # Check the status of the goal		
            goal_status = client.get_state()
		
            # If the goal is active
            if goal_status == GoalStatus.ACTIVE:
                print("Goal is active.")
                
                # Ask if the user wants to cancel the goal
                cancel_input = input("Do you want to cancel the goal? (yes/no): ").lower()
                if cancel_input == "yes":
                    success = cancel_goal(client)
                    # Check if the goal has been canceled successfully
                    if success:
                        print("Goal successfully canceled.")
                        break
                    else:
                        print("Failed to cancel goal.")
                        break  
            # If the goal has been reached            
            elif goal_status == GoalStatus.SUCCEEDED:
                print("Goal is reached.")
                break
            # If the goal has been canceled    
            elif goal_status == GoalStatus.ABORTED:
                print("Goal is canceled.")
                break
            # If the goal is not active    
            elif goal_status != GoalStatus.ACTIVE:
                print("Failed to set the goal.")  
                break  
                 
        # Set up the new goal    
        print("Set a new goal.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        pass

