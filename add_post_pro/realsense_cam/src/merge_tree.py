#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def combine_transformations():
    rospy.init_node('combined_transformations_node')

    # Initialize TF2 buffer and broadcaster
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Specify the frames you want to combine
    frame_a = "D415_depth_optical_frame"
    frame_b = "camera_depth_optical_frame"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Get transformations from the two parts of the tree
            transform_a = tf_buffer.lookup_transform("D415_depth_optical_frame", frame_a, rospy.Time(0), rospy.Duration(1.0))
            transform_b = tf_buffer.lookup_transform("camera_depth_optical_frame", frame_b, rospy.Time(0), rospy.Duration(1.0))

            # Combine transformations as needed
            combined_transform = transform_a
            combined_transform.transform = transform_b.transform * transform_a.transform

            # Publish the combined transformation
            tf_broadcaster.sendTransform(combined_transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform lookup failed: %s", str(e))

        rate.sleep()

if __name__ == '__main__':
    try:
        combine_transformations()
    except rospy.ROSInterruptException:
        pass
