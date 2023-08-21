#!/usr/bin/python

from time import sleep
from os import path

import numpy as np

import rospy
import rospkg
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmJointController:

    def __init__(self, time_step=0.1):

        # Set a publishing rate
        self.time_step = time_step
        self.rate = rospy.Rate(1/time_step)

        # Initialize action client to the denso arm
        self.client = actionlib.SimpleActionClient(
            "/add_post_pro_robot/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self.client.wait_for_server()

        self.load_motion_plan()

    def load_motion_plan(self):
        # Get current package path
        rospack = rospkg.RosPack()
        motion_plan_file = path.join(
            rospack.get_path('add_post_pro_control'),
            'config',
            'manipulator_motion_plan.csv')
        rospy.loginfo('Load motion plan file: ' + motion_plan_file)

        # Load motion plan
        self.motion_plan = np.genfromtxt(motion_plan_file, delimiter=',')
        rospy.loginfo('Loaded motion plan size: ' + str(self.motion_plan.shape))
        self.motion_plan_size = self.motion_plan.shape[1]
        self.motion_step_count = 0

    def move_to_starting_pose(self):

        # Set joint goals
        time_step = 10
        pos_goal = FollowJointTrajectoryGoal()
        pos_goal.trajectory.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        pos_goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=self.motion_plan[:6,0].tolist(),
                time_from_start=rospy.Duration(
                    secs=time_step))
        )
        rospy.loginfo('Motion Step: {}, joint = {}'.format(
                1,
                self.motion_plan[:6,0].tolist()
            ))
        pos_goal.trajectory.header.stamp = rospy.Time.now()

        # Send action goal
        self.client.send_goal(pos_goal)
        self.client.wait_for_result(rospy.Duration.from_sec(time_step))

    def move_arm(self):
        # Set joint goals
        pos_goal = FollowJointTrajectoryGoal()
        pos_goal.trajectory.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        for motion_step in range(self.motion_plan_size-1):
            pos_goal.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=self.motion_plan[:6,motion_step+1].tolist(),
                    time_from_start=rospy.Duration(
                        secs=motion_step*self.time_step))
            )
            rospy.loginfo('Motion Step: {}, joint = {}'.format(
                motion_step+1,
                self.motion_plan[:6,motion_step+1].tolist()
            ))
        pos_goal.trajectory.header.stamp = rospy.Time.now()

        # Send action goal
        self.client.send_goal(pos_goal)
        self.client.wait_for_result(rospy.Duration.from_sec(20.0))


def move_manipulator():

    rospy.init_node('manipulator_joint_input')

    # Initialize kogumi base controller
    arm_controller = ArmJointController(time_step=2.0)

    # Move Arm to initial position
    arm_controller.move_to_starting_pose()

    raw_input('Arm at the starting position. Press enter to continue.')

    # Move Arm
    arm_controller.move_arm()


if __name__=='__main__':
    try:
        move_manipulator()
    except rospy.ROSInterruptException:
        pass