#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pcl_ros import pcl_conversions
from pcl import PointCloud
import pcl



def point_cloud_callback(data):
    rospy.loginfo("Received point cloud with %d points", data.width * data.height)

    # Convert ROS PointCloud2 to PCL PointCloud
    cloud = pcl_conversions.fromROSMsg(data, PointCloud())

    # Voxel Grid Filter
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01  # Define the size of the grid cells
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # Passthrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.1  # Define min point in z-axis
    axis_max = 1.0  # Define max point in z-axis
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: Further processing steps can be added here

    rospy.loginfo("Filtered point cloud has %d points", cloud_filtered.size)






def depth_callback(data):
    rospy.loginfo("Received depth image of size: %d x %d", data.width, data.height)

def main():
    rospy.init_node('depth_processor', anonymous=True)

    rospy.Subscriber("/camera_name/depth/image_raw", Image, depth_callback)

    rospy.Subscriber("/camera_name/depth/points", PointCloud2, point_cloud_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
