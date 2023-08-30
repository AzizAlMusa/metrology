#!/usr/bin/env python

import rospy
import math
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty, Trigger  # Assuming the services you mentioned are of type Empty
import tf.transformations

def move_camera_to_poses():
    # Define the 8 poses
    radius = 1.0  # distance from the cube
    height = 0.5  # height of the camera from the ground
    poses = []

    for i in range(8):
        angle = 2 * math.pi * i / 8
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = height
        yaw = angle + math.pi  # camera facing towards the cube
        poses.append((x, y, z, 0, 0, yaw))

    # Initialize the ROS node
    rospy.init_node('camera_mover')
    
    rospy.loginfo("WAITING for services")
    # Wait for the services to be available
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.wait_for_service('/depth_processor_cpp/capture_measurement')
    rospy.wait_for_service('/depth_processor_cpp/align_measurements')
    rospy.loginfo("SERVICES arrived!")
    # Create service proxies
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    capture_measurement = rospy.ServiceProxy('/depth_processor_cpp/capture_measurement', Trigger)
    align_measurements = rospy.ServiceProxy('/depth_processor_cpp/align_measurements', Trigger)

    for (x, y, z, roll, pitch, yaw) in poses:
        state = SetModelStateRequest()
        state.model_state.model_name = 'camera_robot'
        state.model_state.pose.position.x = x
        state.model_state.pose.position.y = y
        state.model_state.pose.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        state.model_state.pose.orientation.x = quaternion[0]
        state.model_state.pose.orientation.y = quaternion[1]
        state.model_state.pose.orientation.z = quaternion[2]
        state.model_state.pose.orientation.w = quaternion[3]

        # Call the service to move the camera
        resp = set_state(state)
        if not resp.success:
            rospy.logerr("Failed to set camera pose!")
        
        # Capture the measurement
        # capture_measurement()

        # Wait for a while to ensure data is captured
        rospy.sleep(2)

        # Prompt the user to move to the next pose
        raw_input("Press Enter to move to the next pose...")

    # After scanning from all poses, align the measurements
    # align_measurements()

if __name__ == '__main__':
    try:
        rospy.sleep(5)  # Wait for 5 seconds to ensure everything is loaded in Gazebo

        move_camera_to_poses()
    except rospy.ROSInterruptException:
        pass
