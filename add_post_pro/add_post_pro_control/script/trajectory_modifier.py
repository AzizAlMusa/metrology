#!/usr/bin/env python

import rospy
import copy
from moveit_msgs.msg import RobotTrajectory

# Callback function for receiving original trajectory
def callback(data):
    original_trajectory = data.joint_trajectory  # Extract the JointTrajectory part
    
    arm_robot_trajectory = RobotTrajectory()
    turntable_robot_trajectory = RobotTrajectory()
    
    # Assuming there are at least 6 joints, extract the names of the last 6 joints
    arm_robot_trajectory.joint_trajectory.joint_names = original_trajectory.joint_names[-6:]
    

    
    turntable_robot_trajectory.joint_trajectory.joint_names = ["turntable_revolve_joint"]
    
    for i, point in enumerate(original_trajectory.points):
        
        index_joint_0 = original_trajectory.joint_names.index("joint_0")
        
         # Create a new point for the arm trajectory, deep copying only necessary to avoid reference issues
        arm_point = copy.deepcopy(point)
        
        # Extract the positions, velocities, and accelerations of the last 6 joints
        arm_point.positions = point.positions[-6:]
        arm_point.velocities = point.velocities[-6:] if point.velocities else []  # Check if velocities exist
        arm_point.accelerations = point.accelerations[-6:] if point.accelerations else []  # Check if accelerations exist
        
        # Append this new point to the arm trajectory
        arm_robot_trajectory.joint_trajectory.points.append(arm_point)

        # Create new point for the turntable trajectory
        turntable_point = copy.deepcopy(point)
        
        turntable_point.positions = [-point.positions[index_joint_0]]
        turntable_point.velocities = [-point.velocities[index_joint_0]]
        turntable_point.accelerations = [-point.accelerations[index_joint_0]]
        
        # Append this new point to the turntable trajectory
        turntable_robot_trajectory.joint_trajectory.points.append(turntable_point)
    
    # Publish modified trajectories
    arm_publisher.publish(arm_robot_trajectory)
    turntable_publisher.publish(turntable_robot_trajectory)


# Initialize ROS node
rospy.init_node('trajectory_modifier')

# Create publishers for the modified trajectories
arm_publisher = rospy.Publisher('/modified_arm_trajectory', RobotTrajectory, queue_size=10)
turntable_publisher = rospy.Publisher('/modified_turntable_trajectory', RobotTrajectory, queue_size=10)

# Subscribe to the original trajectory topic
rospy.Subscriber('robot_scan_trajectory', RobotTrajectory, callback)

# Keep the node running
rospy.spin()