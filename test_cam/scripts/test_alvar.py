#!/usr/bin/env python

import numpy as np
import rospy
import tf
#from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
marker_arr = MarkerArray()
tf_listener = None
run_marker_callback = True

def cmd_vel_callback(data):
    global run_marker_callback

    run_marker_callback = (abs(data.angular.z) < 0.3)

def marker_callback(data):
    global marker_arr

    # Update markers only if robot isn't spinning that fast
    if run_marker_callback:
        # Transform pose to /map
        new_marker = data
        new_marker.pose = tf_listener.transformPose('/map', data).pose

        new_pos = np.array((new_marker.pose.position.x,
                            new_marker.pose.position.y,
                            new_marker.pose.position.z))

        in_marker_arr = False

        # Look for a match with previously detected markers
        for i, old_marker in enumerate(marker_arr.markers):
            old_pos = np.array((old_marker.pose.position.x,
                                old_marker.pose.position.y,
                                old_marker.pose.position.z))

            err = ((old_pos - new_pos) ** 2).mean()

            tol = 0.1

            if err < tol:
                in_marker_arr = True

                # Update less if high error
                u = (tol - err) ** 4
                marker_arr.markers[i].pose = new_marker.pose

                marker_arr.markers[i].pose.position.x = \
                        new_marker.pose.position.x * u + \
                        old_marker.pose.position.x * (1.0 - u)
                marker_arr.markers[i].pose.position.y = \
                        new_marker.pose.position.y * u + \
                        old_marker.pose.position.y * (1.0 - u)
                marker_arr.markers[i].pose.position.z = \
                        new_marker.pose.position.z * u + \
                        old_marker.pose.position.z * (1.0 - u)
                break

        # If no matches, add this to the currently existing markers
        if not in_marker_arr:
            new_marker.header.frame_id = '/map'
            new_marker.id = len(marker_arr.markers)
            marker_arr.markers.append(new_marker)

def imu_callback(data):
    global run_marker_callback

    run_marker_callback = (abs(data.angular_velocity.z) < 0.1)

def main():
    rospy.init_node('test_alvar', anonymous=True)

    global tf_listener

    if tf_listener is None:
        tf_listener = tf.TransformListener()

    rospy.Subscriber('visualization_marker', Marker, marker_callback)
    #rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('imu', Imu, imu_callback)

    while not rospy.is_shutdown():
        publisher.publish(marker_arr)

    rospy.spin()


if __name__ == '__main__':
    main()

