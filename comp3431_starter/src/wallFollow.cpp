/*
 * wall_follow.cpp
 *
 *  Created on: 15/07/2016
 *      Author: mmcgill
 */

#include <ros/ros.h>

#include <comp3431_starter/wallFollow.hpp>

#define BASE_FRAME "base_link"
#define MAX_SIDE_LIMIT 0.50
#define MIN_APPROACH_DIST 0.30
#define MAX_APPROACH_DIST 0.50

#define ROBOT_RADIUS 0.2

#define MAX_SPEED 0.25
#define MAX_TURN 1.0

#define CLIP_0_1(X) ((X) < 0 ? 0 : (X) > 1 ? 1 : (X))

namespace comp3431 {

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
        ROS_ERROR("Unable to get transformation (tfListener).");
        return;
    }

    // Turn laser scan into a point cloud
    std::vector<tf::Vector3> points;
    points.resize(scan->ranges.size());

    float XMaxLeft = -INFINITY, XMaxRight = INFINITY, XMinFront = INFINITY, angle = scan->angle_min;
    for (int n = 0; n < points.size(); ++n, angle += scan->angle_increment) {
        tf::Vector3 point(cos(angle) * scan->ranges[n], sin(angle) * scan->ranges[n], 0);

        // transfer point to base_link frame
        points[n] = point = transform * point;
        // Get the closest point on our right (aka y < 0)
        if (point.y() < 0 && fabs(point.y()) <= MAX_APPROACH_DIST && fabs(point.x()) <= ROBOT_RADIUS)
            if (point.y() < XMaxRight)
                XMaxRight = point.y();

        // Get the closest point on our left (aka y > 0)
        if (point.y() > 0 && fabs(point.y()) <= MAX_APPROACH_DIST && fabs(point.x()) <= ROBOT_RADIUS)
            if (point.y() > XMaxLeft)
                XMaxLeft = point.y();

        // Get the closest point in front of us
        if (point.x() > 0 && point.x() <= MAX_APPROACH_DIST && fabs(point.y()) <= ROBOT_RADIUS) {
            if (point.x() < XMinFront)
                XMinFront = point.x();
        }

        std::cout << "L: " << XMaxLeft << "\t";
        std::cout << "F: " << XMinFront << "\t";
        std::cout << "R: " << XMaxRight << std::endl;
    }
    XMaxRight *= -1;

    float turn = 0, drive = 0;
    
    if (XMinFront < MAX_APPROACH_DIST) {
    	drive = 0;
	turn = 1;
    } else {
    	drive = 1;
    	turn = XMaxLeft - XMaxRight;
    }

    /*
    if (XMaxLeft > MAX_APPROACH_DIST && XMaxRight > MAX_APPROACH_DIST && XMinFront > MAX_APPROACH_DIST) {
	turn = 0;
    	drive = 1;
    } else if (XMaxLeft > MAX_APPROACH_DIST && XMaxRight > MAX_APPROACH_DIST && XMinFront < MAX_APPROACH_DIST) {
    	turn = 1;
	drive = 0;
    } else if (XMaxLeft > MAX_APPROACH_DIST && XMaxRight < MAX_APPROACH_DIST && XMinFront < MAX_APPROACH_DIST) {
    	turn = 1;
	drive = 0;
    } else if (XMaxLeft < MAX_APPROACH_DIST && XMaxRight < MAX_APPROACH_DIST && XMinFront < MAX_APPROACH_DIST) {
    	turn = 1;
	drive = 0;
    }
    */

    /*
    if (XMaxLeft > MAX_APPROACH_DIST) {
        // no left wall, turn left
        turn = 1;
        drive = 0;
    } else if (XMaxRight > MAX_APPROACH_DIST) {
        // no right wall, turn right
        turn = -1;
        drive = 0;
    } else if (XMinFront < MIN_APPROACH_DIST) {
        // wall in front, turn right
        turn = -1;
        drive = 0;
    } else {
        // go straight
        turn = 0;
        drive = 1;
    }
    */
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
    tf::StampedTransform transform;
    try {
        slamListener.waitForTransform(BASE_FRAME, submap->header.frame_id, submap->header.stamp, ros::Duration(2.0));
        slamListener.lookupTransform(BASE_FRAME, submap->header.frame_id, submap->header.stamp, transform);
    } catch (tf::TransformException& tfe) {
        ROS_ERROR("Unable to get transformation (slamListener).");
        return;
    }
    
    if (MIN_HOME < transform.getOrigin().x() && transform.getOrigin().x() < MAX_HOME &&
        MIN_HOME < transform.getOrigin().y() && transform.getOrigin().y() < MAX_HOME) {
        std::cout << "HOME\n";
        // paused = stopped = true;
    }
}
}  // namespace comp3431
