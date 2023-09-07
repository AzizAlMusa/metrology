#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def attach_frames():
    rospy.init_node('frame_attachment_node')
    broadcaster = tf2_ros.TransformBroadcaster()

    # Define the names of the parent and child frames
    parent_frame = "D415_color_optical_frame"  # The frame to attach to
    child_frame = "camera_depth_optical_frame"    # The frame being attached

    rate = rospy.Rate(10.0)  # Publishing rate: 10 Hz

    while not rospy.is_shutdown():
        try:
            # Create a TransformStamped message
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = parent_frame
            transform.child_frame_id = child_frame

            # Define the fixed transformation (adjust as needed)
            transform.transform.rotation.w = 1.0     # Adjust the rotation values

            # Publish the fixed transformation
            broadcaster.sendTransform(transform)
        except rospy.ROSException as e:
            rospy.logerr("Error while publishing transformation: %s", str(e))

        rate.sleep()

if __name__ == '__main__':
    try:
        attach_frames()
    except rospy.ROSInterruptException:
        pass

   