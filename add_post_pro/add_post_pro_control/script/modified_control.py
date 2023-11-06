#!/usr/bin/env python

import rospy
from moveit_msgs.msg import RobotTrajectory
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Global variables to hold the buffered trajectories
arm_trajectory = None
turntable_trajectory = None

# Callback function for the arm
def arm_callback(data):
    global arm_trajectory
    arm_trajectory = data
    execute_synchronized_motion()

# Callback function for the turntable
def turntable_callback(data):
    global turntable_trajectory
    turntable_trajectory = data
    execute_synchronized_motion()

# Function to execute synchronized motion
def execute_synchronized_motion():
    global arm_trajectory, turntable_trajectory
    

    if arm_trajectory is not None and turntable_trajectory is not None:

        # Initialize SimpleActionClients
        arm_client = SimpleActionClient('/add_post_pro_robot_b120/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # turntable_client = SimpleActionClient('/add_post_pro_equiv/turntable_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # real_turntable_client = SimpleActionClient('/physical_turntable/turntable_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
      
        # Wait for servers to be ready
        arm_client.wait_for_server()
        # turntable_client.wait_for_server()
        # real_turntable_client.wait_for_server()

        # Create goals from buffered trajectories
        arm_goal = FollowJointTrajectoryGoal(trajectory=arm_trajectory.joint_trajectory)
        # turntable_goal = FollowJointTrajectoryGoal(trajectory=turntable_trajectory.joint_trajectory)
        # real_turntable_goal = FollowJointTrajectoryGoal(trajectory=turntable_trajectory.joint_trajectory)

        # Send Goals
        arm_client.send_goal(arm_goal)
        # turntable_client.send_goal(turntable_goal)
        # real_turntable_client.send_goal(real_turntable_goal)

        # Wait for both to finish
        arm_client.wait_for_result()
        # turntable_client.wait_for_result()
        # real_turntable_client.wait_for_result()
        
        # Clear the buffered trajectories
        arm_trajectory = None
        turntable_trajectory = None
        # real_turntable_client = None

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('trajectory_direct_control', anonymous=True)

    # Create subscribers
    rospy.Subscriber("/modified_arm_trajectory", RobotTrajectory, arm_callback)
    rospy.Subscriber("/modified_turntable_trajectory", RobotTrajectory, turntable_callback)

    rospy.spin()
