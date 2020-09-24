# SLAM

### gmapping

`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`

### hector

`sudo apt-get install ros-melodic-hector-slam`\
`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector`

### cartographer

`sudo apt-get install ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-ros-msgs ros-melodic-cartographer-rviz`\
Find `~/catkin_ws/src/turtlebot3/turtlebot3_slam/config/turtlebot3_lds_2d.lua` and change `tracking_frame = "imu_link",` to `tracking_frame = "base_footprint",`\
`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer`
