#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import PointCloud2, PointStamped
from sensor_msgs import point_cloud2

# Create a TransformListener object to listen for transform updates
listener = tf.TransformListener()

# Create a PointCloud2 subscriber
def callback(pc2):
    # Wait for the transform from the pointcloud frame to the target frame to become available
    listener.waitForTransform(pc2.header.frame_id, 'true_laser', rospy.Time(), rospy.Duration(4.0))

    # Create an empty list to store the transformed points
    points_transformed = []

    # Loop through each point in the pointcloud
    for p in point_cloud2.read_points(pc2, skip_nans=True):
        # Convert the point to a PointStamped object
        p_stamped = PointStamped()
        p_stamped.header.frame_id = pc2.header.frame_id
        p_stamped.point.x = p[0]
        p_stamped.point.y = p[1]
        p_stamped.point.z = p[2]

        # Use the transformPoint() method to transform the point to the target frame
        p_transformed = listener.transformPoint('target_frame', p_stamped)

        # Add the transformed point to the list
        points_transformed.append([p_transformed.point.x, p_transformed.point.y, p_transformed.point.z])

    # Create a new PointCloud2 message with the transformed points
    pc2_transformed = PointCloud2()
    pc2_transformed.header.stamp = rospy.Time.now()
    pc2_transformed.header.frame_id = 'target_frame'
    pc2_transformed.height = 1
    pc2_transformed.width = len(points_transformed)
    pc2_transformed.fields = pc2.fields
    pc2_transformed.is_bigendian = pc2.is_bigendian
    pc2_transformed.point_step = 12
    pc2_transformed.row_step = 12 * len(points_transformed)
    pc2_transformed.data = bytearray([int(x) for point in points_transformed for x in point])

    # Publish the transformed pointcloud
    pub.publish(pc2_transformed)

# Create a PointCloud2 subscriber
rospy.init_node('transform_pointcloud')
sub = rospy.Subscriber('input_pointcloud', PointCloud2, callback)

# Create a PointCloud2 publisher
pub = rospy.Publisher('output_pointcloud', PointCloud2, queue_size=10)

# Spin until Ctrl-C is pressed
rospy.spin()
