/*
 * wall_follow.cpp
 *
 *  Created on: 15/07/2016
 *      Author: mmcgill
 */

#include <ros/ros.h>

#include <comp3431_starter/wallFollow.hpp>

#define CLIP(X) ((X) < 0 ? 0 : (X) > 1 ? 1 : (X))

namespace {
bool d_cmp(double a, double b, double eps) {
    return std::abs(a - b) <= eps ||
           std::abs(a - b) < (std::fmax(std::abs(a), std::abs(b)) * eps);
}
}  // namespace

namespace comp3431 {

constexpr char BASE_FRAME[10] = "base_link";
constexpr float MAX_SIDE_LIMIT = 0.50;
constexpr float MIN_APPROACH_DIST = 0.30;
constexpr float MAX_APPROACH_DIST = 0.50;
constexpr float ROBOT_RADIUS = 0.2;
constexpr float MAX_SPEED = 0.25;
constexpr float MAX_TURN = 1.0;
constexpr float MIN_HOME = -0.1;
constexpr float MAX_HOME = 0.1;

WallFollower::WallFollower() : paused(true), stopped(false), side(LEFT) {
}

void WallFollower::configure() {
}

void WallFollower::startup() {
    ros::NodeHandle nh;
    scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &WallFollower::callbackScan, this);
    commandSub = nh.subscribe<std_msgs::String>("cmd", 1, &WallFollower::callbackControl, this);
    slamSub = nh.subscribe<cartographer_ros_msgs::SubmapList>("submap_list", 1, &WallFollower::callbackSlam, this);
    twistPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
}

void WallFollower::shutdown() {
    if (!paused) {
        paused = true;
        geometry_msgs::Twist t;
        t.linear.x = t.linear.y = t.linear.z = 0;
        t.angular.x = t.angular.y = t.angular.z = 0;

        twistPub.publish(t);
    }
}

void WallFollower::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
    if (paused) {
        if (!stopped) {
            geometry_msgs::Twist t;
            t.linear.x = t.linear.y = t.linear.z = 0;
            t.angular.x = t.angular.y = t.angular.z = 0;

            twistPub.publish(t);
            stopped = true;
        }
        return;
    }

    tf::StampedTransform transform;
    // get laser to base_link transform
    try {
        tfListener.waitForTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, ros::Duration(2.0));
        tfListener.lookupTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, transform);
    } catch (tf::TransformException& tfe) {
        ROS_ERROR("Unable to get transformation.");
        return;
    }

    // Turn laser scan into a point cloud
    std::vector<tf::Vector3> points;
    points.resize(scan->ranges.size());

    float left = -INFINITY, right = INFINITY, front = INFINITY, angle = scan->angle_min;
    for (int n = 0; n < points.size(); ++n, angle += scan->angle_increment) {
        tf::Vector3 point(cos(angle) * scan->ranges[n], sin(angle) * scan->ranges[n], 0);

        // transfer point to base_link frame
        points[n] = point = transform * point;
        // Get the closest point on our right (aka y < 0)
        if (point.y() < 0 && fabs(point.y()) <= MAX_APPROACH_DIST && fabs(point.x()) <= ROBOT_RADIUS)
            if (point.y() < right)
                right = point.y();

        // Get the closest point on our left (aka y > 0)
        if (point.y() > 0 && fabs(point.y()) <= MAX_APPROACH_DIST && fabs(point.x()) <= ROBOT_RADIUS)
            if (point.y() > left)
                left = point.y();

        // Get the closest point in front of us
        if (point.x() > 0 && point.x() <= MAX_APPROACH_DIST && fabs(point.y()) <= ROBOT_RADIUS) {
            if (point.x() < front)
                front = point.x();
        }
    }
    right *= -1; // make value positive

    // debugging info
    std::cout << "L: " << left << "\t";
    std::cout << "F: " << front << "\t";
    std::cout << "R: " << right << std::endl;

    float turn = 0, drive = 0;

    if (left == -INFINITY) {
        turn = 1;
        drive = 0;
    } else if (front <= MIN_APPROACH_DIST) {
        // if (left < right) {
            turn = -1;
            drive = 0;
        // } else if (left > right) {
            // turn = 1;
            // drive = 0;
        // }
    } else {
        // if (left < right) {
            float turn1 = CLIP((ROBOT_RADIUS - left) / (2 * ROBOT_RADIUS));
            float turn2 = CLIP((MAX_APPROACH_DIST - front) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST));
            float drive1 = CLIP((ROBOT_RADIUS + left) / (2 * ROBOT_RADIUS));
            float drive2 = CLIP((front - MIN_APPROACH_DIST) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST));
            turn = turn1 - turn2;
            drive = drive1 * drive2;
        // } else if (left > right) {
        //     float turn1 = CLIP((ROBOT_RADIUS - right) / (2 * ROBOT_RADIUS));
        //     float turn2 = CLIP((MAX_APPROACH_DIST - front) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST));
        //     float drive1 = CLIP((ROBOT_RADIUS + right) / (2 * ROBOT_RADIUS));
        //     float drive2 = CLIP((front - MIN_APPROACH_DIST) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST));
        //     turn = turn1 - turn2;
        //     drive = drive1 * drive2;
        // }
    }
    
    // publish twist
    geometry_msgs::Twist t;
    t.linear.y = t.linear.z = 0;
    t.linear.x = drive * MAX_SPEED;
    t.angular.x = t.angular.y = 0;
    t.angular.z = turn * MAX_TURN;
    twistPub.publish(t);
    stopped = false;
}

void WallFollower::callbackControl(const std_msgs::StringConstPtr& command) {
    ROS_INFO("Recieved %s message.\n", command->data.c_str());
    if (strcasecmp(command->data.c_str(), "start") == 0 ||
        strcasecmp(command->data.c_str(), "go") == 0 ||
        strcasecmp(command->data.c_str(), "begin") == 0 ||
        strcasecmp(command->data.c_str(), "resume") == 0) {
        paused = false;
    } else if (strcasecmp(command->data.c_str(), "stop") == 0 ||
               strcasecmp(command->data.c_str(), "pause") == 0 ||
               strcasecmp(command->data.c_str(), "halt") == 0) {
        paused = true;
    } else if (strcasecmp(command->data.c_str(), "toggle") == 0) {
        paused = !paused;
    }
}

void WallFollower::callbackSlam(const cartographer_ros_msgs::SubmapListConstPtr& submap) {
    if (d_cmp(submap[0]->pose.point.x, submap[end]->pose.point.x, __FLT_EPSILON__) &&
        d_cmp(submap[0]->pose.point.y, submap[end]->pose.point.y, __FLT_EPSILON__) &&
        d_cmp(submap[0]->pose.point.z, submap[end]->pose.point.z, __FLT_EPSILON__))
        stopped = paused = true;
}

void WallFollower::~WallFollower(const std_msgs::Car) {
}

}  // namespace comp3431
