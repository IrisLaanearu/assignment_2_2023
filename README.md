# Assignment 2 
# ROS Package: assignment_2_2023

## Assignment description
Create a new package, in which you will develop three nodes: <br>

(a) A node that implements an actionclient, allowing the user to set a target (x,y) or to cancel it. Use the feedback/status of the 
action server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x,y,vel_x,vel_z),
by relying on the values published on the topic /odom; <br>

(b) A service node that, when called, returns the coordinates of the last target sent by the user; <br>

(c) A service node that subscribes to the RobotState and implements a server to retrieve the distance of the robot from the target and the 
robot’s average speed; <br>

(d) Create a launch file to start the whole simulation. Use a parameter to select the size of the averaging window of node (c). <br>

## Installing and running
First create a ROS workspace.
```bash
$ -mkdir–pmy_ros/src
```
Buld the project.
```bash
$ catkin_make
```
Clone this package.
```bash
$ git clone https://github.com/IrisLaanearu/assignment_2_2023.git
```
To run the package and launch the simulation move to the src folder of your ROS workspace and use the launch file assignment1.launch.
```bash
$ roslaunch assignment_2_2023 assignment1.launch
```
To run the nodes (b) last_target and (c) distance_from_goal use rosservice call. Move to the package folder in another terminal window and run it with argument True.
Last_target node will return the x and y coordinates of the last target set by the user.
```bash
$ rosservice call last_target True
```
Distance_from_goal node will return the distane of the robot from the goal and the average speed of the window set in the parameter of the launch file.
```bash
$ rosservice call distance_from_goal True
```

## Flowchart of the node (a) user_interface
1. Main function <br>
![Main_function](https://github.com/IrisLaanearu/assignment_2_2023/assets/145934148/98508392-9662-4f6b-b282-fd3ad56a4402)

2. Set goal function <br>
![image](https://github.com/IrisLaanearu/assignment_2_2023/assets/145934148/75645cf6-4ed8-40af-bc73-fef4b1d747b7)

3. Cancel goal function <br>
![cancel goal](https://github.com/IrisLaanearu/assignment_2_2023/assets/145934148/9670b158-7efb-4c48-b866-492df8b04f02)

4. Odom callback function <br>
![image](https://github.com/IrisLaanearu/assignment_2_2023/assets/145934148/b4d0b0e3-191c-4779-926a-d837bd122ead)

## Graph of the communication between processes <br>
![image](https://github.com/IrisLaanearu/assignment_2_2023/assets/145934148/aff33c64-10d7-41d1-83b6-1e12b1615e3a)

## Possible improvements
The user_interface node could check the status of the goal all the time but currently it only checks if the user inserts no to the question "Do you want to cancel the goal?". 





