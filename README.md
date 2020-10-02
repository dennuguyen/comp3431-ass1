# Project 1: TurtleBot3 Mapping and Navigation

Assignment Specification:\
https://webcms3.cse.unsw.edu.au/COMP3431/20T3/resources/52289

COMP3431 Course Repository:\
http://robolab.cse.unsw.edu.au:4443/comp3431-rsa/comp3431-rsa

## Summary of Specifications

### Task

1. Explore world.
2. Locate AR markers.
3. Build map.
4. Place AR markers on map.
5. Plot robot path on map.
6. Return robot to starting location when all AR markers are located.
7. Complete task as quickly as possible.

### Deliverables

1. ROS package in C++ or Python (everyything required to run the program).
2. Live demonstration: world map, AR marker locations, robot path.
    1. We can have as many live demonstrations as we want, the best run will count towards our final mark.
3. Report (<= 5 pages):
    1. How we solve the task.
    2. ROS packages we chose to use and why.
    3. Evaluation and explanation of our performance during live demonstration.
    4. Limitations of our software.

### Criteria

- Quality and accuracy of map.
- Accuracy of AR marker placements.
- Accuracy of recorded robot path.
- Accuracy and ability of robot to return to its starting location.
- Speed at which robot completes the task.

Mark penalties:
- Interacting with autonomous software (other than start/stop commands).
- Robot collision with maze or markers.
- Interfering with runs of other groups.

## Git Maintenance

To get changes from the official COMP3431 course repo: `git pull upstream master`

# WARNING
Do not unplug before shutting down

Run sudo shutdown now on the turtlebot before unplugging


## CONNECTING TO THE TURTLEBOT

1. Connect to the hidden network
user : unsw_robotics
pass : newSOUTHrobotics
WPA2-Personal

2. ssh ubuntu@<robot_name>
pass : rsa2020

3. Modify .bashrc on the turtlebot so ROS_MASTER_URI points to the
remote PC and ROS_HOSTNAME points to localhost. Use ifconfig to find
the IP addresses

ROS_MASTER_URI=xxx.xxx.xxx.xxx
ROS_HOSTNAME=xxx.xxx.xxx.xxx

4. Source .bashrc and catkin_ws/devel/setup.bash on both the remote
PC and the turtlebot

5. Run roscore on the remote PC

6. Run roslaunch turtlebot3_bringup turtlebot3_robot.launch on the
turtlebot

7. Run roslaunch turtlebot3_bringup turtlebot3_remote.launch on the
remote PC

8. Do stuff

### Running Simulation
`roslaunch turtlebot3_gazebo maze.world`\
`roslaunch turtlebot3_slam turtlebot_slam.launch slam_methods:=cartographer`\
`roslaunch comp3431_starter wallFollow.launch`\
`rostopic pub -1 /cmd std_msgs/String -- 'start'`
