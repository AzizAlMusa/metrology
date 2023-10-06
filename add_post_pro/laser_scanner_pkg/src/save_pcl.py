#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from pcl import PointCloud, io

def save_pointcloud(request):
    pcl_cloud = PointCloud()
    pcl_cloud.from_msg(request.pointcloud)

    filename = "/home/cerlab/Desktop/aman/Files/output_pointcloud.stl"  # Replace with your desired location and filename

    io.savePolygonFileSTL(filename, pcl_cloud)

    return {"success": True, "message": "Point cloud saved"}

if __name__ == "__main__":
    rospy.init_node("pointcloud_saver_service")

    service = rospy.Service("save_pointcloud_service", Trigger, save_pointcloud)

    rospy.spin()