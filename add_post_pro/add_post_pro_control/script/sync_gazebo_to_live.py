#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class JointStateController:
    def __init__(self):
        rospy.init_node('joint_state_controller')

        # Initialize action client
        self.client = actionlib.SimpleActionClient(
            '/add_post_pro_equiv/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        self.client.wait_for_server()

        # Subscriber to the robot_b120 joint states
        rospy.Subscriber('/add_post_pro_robot_b120/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, data):
        # Check if we have enough joints, else return
        if len(data.position) < 6:
            rospy.logwarn("Not enough joint states received.")
            return

        # Create a new JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = [0] + list(data.position[-6:])  # Keep first joint at 0, take the last 6 joints
        point.time_from_start = rospy.Duration(1.0)  # Set a time for the trajectory point

        # Create a new JointTrajectoryGoal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        goal.trajectory.points.append(point)

        # Send the goal to the action server
        self.client.send_goal(goal)
        rospy.loginfo("Goal sent to action server.")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = JointStateController()
        controller.spin()
    except rospy.ROSInterruptException:
        pass
