#!/usr/bin/env python

import rospy
import copy
from moveit_msgs.msg import RobotTrajectory

# Callback function for receiving original trajectory
def callback(data):
    original_trajectory = data.joint_trajectory  # Extract the JointTrajectory part
    
    arm_robot_trajectory = RobotTrajectory()
    turntable_robot_trajectory = RobotTrajectory()
    
    arm_robot_trajectory.joint_trajectory = copy.deepcopy(original_trajectory)
    
    arm_robot_trajectory.joint_trajectory.joint_names = original_trajectory.joint_names
    
    turntable_robot_trajectory.joint_trajectory.joint_names = ["turntable_revolve_joint"]
    
    for i, point in enumerate(original_trajectory.points):
        
        index_joint_0 = original_trajectory.joint_names.index("joint_0")
        
        # Create new lists from tuples and modify them
        new_positions = list(point.positions)
        new_velocities = list(point.velocities)
        new_accelerations = list(point.accelerations)
        
        # Make joint_0 stationary for arm
        new_positions[index_joint_0] = 0.0
        new_velocities[index_joint_0] = 0.0
        new_accelerations[index_joint_0] = 0.0
        
        # Reassign the modified lists back to the arm trajectory
        arm_robot_trajectory.joint_trajectory.points[i].positions = tuple(new_positions)
        arm_robot_trajectory.joint_trajectory.points[i].velocities = tuple(new_velocities)
        arm_robot_trajectory.joint_trajectory.points[i].accelerations = tuple(new_accelerations)
        
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
