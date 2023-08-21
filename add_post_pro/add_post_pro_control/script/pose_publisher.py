#!/usr/bin/env python


# First, import the necessary modules
import rospy
import tf
import geometry_msgs.msg

# Initialize the ROS node with the desired name
rospy.init_node("arm_pose")

# Initialize the ROS publisher
publisher = rospy.Publisher("arm_pose/pose", geometry_msgs.msg.PoseStamped, queue_size=10)

# Initialize the tf listener
tf_listener = tf.TransformListener()

# Set the reference frame and target frame
reference_frame = "/world"
target_frame = "/true_laser"

# Set the rate at which to publish the pose
#rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    # Get the transformation from the target frame to the reference frame
     
    try:
        (trans, rot) = tf_listener.lookupTransform( reference_frame, target_frame, rospy.Time(0))
       
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       
        continue

    # Create a pose message
    pose_msg = geometry_msgs.msg.PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = reference_frame
    pose_msg.pose.position = geometry_msgs.msg.Point(*trans)
    pose_msg.pose.orientation = geometry_msgs.msg.Quaternion(*rot)

    # Publish the pose message
    publisher.publish(pose_msg)

    # Sleep to maintain the desired rate
    #rate.sleep()
