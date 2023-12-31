/*
 * controlWrapper.hpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#ifndef COMP3431_STARTER_WALLFOLLOW_HPP_
#define COMP3431_STARTER_WALLFOLLOW_HPP_

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <chrono>
#include <sstream>

namespace comp3431 {

constexpr float MIN_HOME = -0.6;
constexpr float MAX_HOME = 0.6;

class WallFollower {
   private:
    bool paused, stopped;

    ros::Subscriber scanSub, commandSub, slamSub, odomSub;
    ros::Publisher twistPub;
    tf::TransformListener tfListener;
    std::chrono::time_point<std::chrono::system_clock> start;

    enum Side { LEFT,
                RIGHT };
    Side side;

   public:
    WallFollower();
    virtual ~WallFollower(){};

    void configure();
    void startup();
    void shutdown();

    void callbackScan(const sensor_msgs::LaserScanConstPtr& scan);
    void callbackControl(const std_msgs::StringConstPtr& command);
    void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom);
};

}  // namespace comp3431

#endif /* COMP3431_STARTER_WALLFOLLOW_HPP_ */

