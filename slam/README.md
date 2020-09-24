# SLAM

### gmapping

`roslaunch turtlebot3_slam turtlebo3_slam.launch slam_methods:=gmapping`

### hector

`sudo apt-get install ros-melodic-hector-slam`\
`roslaunch turtlebot3_slam turtlebo3_slam.launch slam_methods:=hector`

### cartographer

`sudo apt-get install ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-ros-msgs ros-melodic-cartographer-rviz`\
Find `~/catkin_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_cartographer.launch` and change `default="turtlebot3_lds_2d.lua` to `default="turtlebot3_lds_2d_gazebo.lua`\
`roslaunch turtlebot3_slam turtlebo3_slam.launch slam_methods:=cartographer`
