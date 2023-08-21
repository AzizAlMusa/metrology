#!/usr/bin/env python


import rospy

from math import cos,sin

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import  PoseStamped, PointStamped, Point32
from sensor_msgs.msg import PointCloud2, PointCloud, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


import tf
import numpy as np
import ros_numpy
import pdb

class LaserScanner:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("laser_scanner")

        # Subscribe to the laser scan and pose topics
        self.laser_scan_sub = rospy.Subscriber("arm/laser_scan", LaserScan, self.laser_scan_callback)
        self.pose_sub = rospy.Subscriber("arm_pose/pose", PoseStamped, self.pose_callback)

        # Initialize the class properties
        self.laser_scan = None
        self.pose = None
        self.cartesian_points = []
        self.cumulative_cartesian_points = []

        

        self.pointcloud = PointCloud()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        #self.pointcloud.height = 1
        self.pointcloud.header = header
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        #PointCloud publisher
        self.cloud_pub =  rospy.Publisher("laser_cloud", PointCloud, queue_size=10)


    def laser_scan_callback(self, msg):
        # Store the laser scan message in a class property
        self.laser_scan = msg

    def pose_callback(self, msg):
        # Store the pose message in a class property
        self.pose = msg

    def map_to_cartesian(self):

        # Check if the laser scan and pose are available
        if self.laser_scan is None or self.pose is None:
           
            return

        # Map the laser scan ranges to cartesian points
        ranges = self.laser_scan.ranges
        angle_min = self.laser_scan.angle_min
        angle_increment = self.laser_scan.angle_increment
        self.cartesian_points = []
        for r in ranges:
            # Skip invalid ranges
            if r == float("inf") or r == float("nan"):
                continue

            # Calculate the angle and x, y coordinates of the point
            angle = angle_min + len(self.cartesian_points) * angle_increment
            z = r * cos(angle)
            y = r * sin(angle)

            #Transform the point to the reference frame of the laser scanner
            #current_time =rospy.Time(0)
            point = PointStamped()
            point.header.frame_id = "true_laser"
            #point.header.stamp = current_time
            point.point.x = 0
            point.point.y = y
            point.point.z = z

            self.pose.header.frame_id
            import time

           
           
           
           
            try:
                start = time.time()
                self.tf_listener.waitForTransform(self.pose.header.frame_id, '/true_laser', rospy.Time(0), rospy.Duration(0.01))
                #self.tf_listener.lookupTransform(self.pose.header.frame_id, '/true_laser', rospy.Time(0))
                point = self.tf_listener.transformPoint(self.pose.header.frame_id, point)
                end = time.time()
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Unable to transform point")
                continue
            
            #print(end - start)
            #pdb.set_trace()

            #point cloud 1 solution
            self.cartesian_points.append(point)
            self.pointcloud.points.append(Point32(point.point.x, point.point.y, point.point.z))


        self.cloud_pub.publish(self.pointcloud)
       



       
      
    def aggregate_pointcloud(self):
        
        

        header = Header()
        header.frame_id = "world"
        header.stamp = rospy.Time.now()
        # msg = PointCloud2()
        # # Fill in the fields of the message
        # msg.header.stamp = rospy.Time.now()
        # msg.header.frame_id = "world"
        # msg.width = len(self.cartesian_points)
        # msg.height = 1
        # msg.fields = [
        fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        # msg.is_bigendian = False
        # msg.point_step = 12
        # msg.row_step = msg.point_step * msg.width
        # msg.is_dense = False
        # Populate the point cloud data
        # point_cloud_data = np.array([[point.point.x, point.point.y, point.point.z] for point in self.cumulative_cartesian_points], dtype=np.float32)

        # pc2 = point_cloud2.create_cloud(header, fields, point_cloud_data)
        #pdb.set_trace()
        pc2 = ros_numpy.msgify(PointCloud, self.data)
        pdb.set_trace()
        self.cloud_pub.publish(pc2)
        #return pc2
        # for i in range(msg.width):
        #     points.append([float(i), float(i), float(i)] )
        #     msg.data = np.asarray(points, np.float32).tostring()







def main():
   
    laser_scanner = LaserScanner()
    #rate = rospy.Rate(60)  # 0.5 Hz = 2 seconds
    while not rospy.is_shutdown():
       
        laser_scanner.map_to_cartesian()
      
        #rate.sleep()



if __name__ == "__main__":
    main()




