import rospy
from sensor_msgs.msg import PointCloud2, PointField
import struct
import ros_numpy
def callback(msg):
    # Create a generator for the point cloud data
    gen = pc2.read_points(msg, skip_nans=True)

    # Loop through each point in the point cloud
    for point in gen:
        # Unpack the x, y, z coordinates of the current point
        x, y, z = point[:3]
        
        # Print the x, y, z coordinates of the current point
        print("x = %f, y = %f, z = %f" % (x, y, z))

if __name__ == '__main__':
    rospy.init_node('point_cloud_reader')
    rospy.Subscriber('point_cloud_topic', PointCloud2, callback)
    rospy.spin()
