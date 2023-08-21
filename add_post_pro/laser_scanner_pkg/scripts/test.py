#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import ros_numpy
import tf
import tf2_ros

import pdb
def read_points_from_file(filename):
    points = []
    with open(filename, 'r') as f:
        for line in f:
            pdb.set_trace()
            x, y, z = map(float, line.split())
            points.append([x, y, z])
      
    return np.array(points)

def create_pointcloud_msg(points):
    msg = PointCloud2()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/true_laser"


    msg.height = 1
    msg.point_step = 32
    msg.width = len(points) // msg.point_step
    msg.is_dense = True
    msg.is_bigendian = False

    msg.fields.append(PointField('x', 0, PointField.FLOAT32, 1))
    msg.fields.append(PointField('y', 4, PointField.FLOAT32, 1))
    msg.fields.append(PointField('z', 8, PointField.FLOAT32, 1))
    msg.fields.append(PointField('intensity', 16, PointField.FLOAT32, 1))
   
    msg.row_step = msg.point_step * msg.width
    
    
    msg.data = np.asarray(points, np.uint8).tolist()

    return msg

if __name__ == '__main__':
    rospy.init_node('pointcloud_publisher')
    pub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=10)

    filename = './data/minimal.txt'
    # Open the file for reading
    data = []
    with open(filename, 'r') as f:
        # Read the contents of the file
        contents = f.read()
        # Convert the string representation of the list to an actual list
        data = eval(contents)

    #data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    pointcloud_msg = create_pointcloud_msg(data)

   
    # something = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud_msg)
    print(pointcloud_msg)
    

    # # Create a transform listener object
    # listener = tf.TransformListener()


    # # Get the list of frame strings
    # frame_strings = listener.getFrameStrings()
 

    # t = tf.TransformListener(True, rospy.Duration(10.0))
    # while not rospy.is_shutdown():
    #     rospy.loginfo("list: %s", t.getFrameStrings())
  
    # Get the list of frame strings
    # frame_strings = buffer.get_frame_strings()

    # # Print the list of frame strings
    # print(frame_strings)
    rate = rospy.Rate(180)  # set the rate to 30 Hz
    while not rospy.is_shutdown():
        pub.publish(pointcloud_msg)
        rate.sleep()