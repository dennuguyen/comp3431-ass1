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

### Sample Code
## Exploration - Josh and James
-wallFollow code
    Installation
    
    cd ~/catkin_ws/src
    git clone http://robolab.cse.unsw.edu.au:4443/comp3431-rsa/comp3431-rsa.git
    cd ~/catkin_ws
    catkin_make
    source ~/catkin_ws/devel/setup.bash
    
    Implementation - in Gazebo                      
    source ~/catkin_ws/devel/setup.bash                 
    roslaunch comp3431_starter wallFollow.launch                        # starting wallFollow
    
    source ~/catkin_ws/devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch                 
    
    rostopic pub -1 /cmd std_msgs/String -- 'start'                     # telling wallFollow to run
    
    source ~/catkin_ws/devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch           # using rviz in Gazebo
                                                                        # "rosrun rviz rviz" irl?

