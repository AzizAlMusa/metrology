#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import LinkStates
from tf.transformations import quaternion_multiply, quaternion_from_euler

def link_state_callback(msg):
    try:
        index = msg.name.index("camera_robot::camera_link")
        pose = msg.pose[index]

        # Extract the current orientation of the camera_link
        q1 = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Define a fixed rotation to align the camera_link's axes as required
        # q2 = quaternion_from_euler(0, -1.5708, 0)  # -180 degrees around x and -90 degrees around z

        # q_combined = quaternion_multiply(q2, q1)

        # q3 = quaternion_from_euler(1.5708, 0, 0)  # -180 degrees around x and -90 degrees around z
        # Combine the rotations
        # q_final = quaternion_multiply(q3, q_combined)

        broadcaster.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            q1,
            rospy.Time.now(),
            "camera_link",
            "world"
        )
    except ValueError:
        pass

rospy.init_node('camera_link_tf_publisher')

broadcaster = tf.TransformBroadcaster()

# Subscribe to the Gazebo LinkStates topic
rospy.Subscriber("/gazebo/link_states", LinkStates, link_state_callback)

rospy.spin()
