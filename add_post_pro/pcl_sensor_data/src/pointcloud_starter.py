#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header

if __name__ == '__main__':
    '''
    Publishes example pointcloud
    '''

    rospy.init_node('point_cloud_tester')
    pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", PointCloud)
    rospy.loginfo("pcl_publish_example")

    #giving some time for the publisher to register
    rospy.sleep(0.5)

    #declaring pointcloud
    result_pointcloud = PointCloud()

    #filling pointcloud header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'
    result_pointcloud.header = header
    #filling some points
    result_pointcloud.points.append(Point32(1.0, 1.0, 0.0))
    result_pointcloud.points.append(Point32(2.0, 2.0, 0.0))
    result_pointcloud.points.append(Point32(3.0, 3.0, 0.0))
    #publish
    

    rate = rospy.Rate(2)  # 0.5 Hz = 2 seconds
    while not rospy.is_shutdown():
       
        pointcloud_publisher.publish(my_awesome_pointcloud)


        rate.sleep()
        
        
    #exit. we are done!
