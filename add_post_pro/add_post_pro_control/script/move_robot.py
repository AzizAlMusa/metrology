#!/usr/bin/python

from time import sleep
from os import path

import numpy as np

import rospy
import rospkg
import actionlib

import pdb

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint



class ArmJointController:

    def __init__(self, time_step=0.1):

        # Set a publishing rate
        self.time_step = time_step
        self.rate = rospy.Rate(1/time_step)

        # Initialize action client to the denso arm
        self.client = actionlib.SimpleActionClient(
            "/add_post_pro_equiv/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        self.client2 = actionlib.SimpleActionClient(
            "/add_post_pro_equiv/turntable_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        self.client.wait_for_server()
        self.client2.wait_for_server()


        self.load_motion_plan()

    def load_motion_plan(self):
        # Get current package path
        rospack = rospkg.RosPack()
        motion_plan_file = path.join(
            rospack.get_path('add_post_pro_control'),
            'config',
            'new_waypoints.csv')
        rospy.loginfo('Load motion plan file: ' + motion_plan_file)

        sweep_plan_file = path.join(
            rospack.get_path('add_post_pro_control'),
            'config',
            'sweep.csv')
        rospy.loginfo('Load sweep plan file: ' + sweep_plan_file)
        # Load motion plan
        self.motion_plan = np.genfromtxt(motion_plan_file, delimiter=',')
        self.sweep_plan =  np.genfromtxt(sweep_plan_file, delimiter=',')

        rospy.loginfo('Loaded motion plan size: ' + str(self.motion_plan.shape))
        #pdb.set_trace()
        self.motion_plan_size = self.motion_plan.shape[0]
        self.motion_step_count = 0

        self.sweep_plan_size = len(self.sweep_plan)
        self.motion_step_count = 0

    def move_to_starting_pose(self):

        # Set joint goals
        time_step = 1.0
        pos_goal = FollowJointTrajectoryGoal()
        pos_goal.trajectory.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        pos_goal2 = FollowJointTrajectoryGoal()
        pos_goal2.trajectory.joint_names = [
            'turntable_revolve_joint'
        ]

        pos_goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=self.motion_plan[0,1:7].tolist(),
                time_from_start=rospy.Duration(
                    secs=time_step))
        )

        pos_goal2.trajectory.points.append(
            JointTrajectoryPoint(
                positions=self.motion_plan[0,0:1].tolist(),
                time_from_start=rospy.Duration(
                    secs=time_step))
        )
        rospy.loginfo('Motion Step: {}, joint = {}'.format(
                1,
                self.motion_plan[0,1:7].tolist()
            ))
        pos_goal.trajectory.header.stamp = rospy.Time.now()

        # Send action goal
        self.client.send_goal(pos_goal)
        self.client2.send_goal(pos_goal2)

        self.client.wait_for_result(rospy.Duration.from_sec(time_step))
        self.client2.wait_for_result(rospy.Duration.from_sec(time_step))

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
        
        pos_goal2 = FollowJointTrajectoryGoal()
        pos_goal2.trajectory.joint_names = [
            'turntable_revolve_joint'
        ]

        for motion_step in range(self.motion_plan_size-1):
            pos_goal.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=self.motion_plan[motion_step+1, 1:7].tolist(),
                    time_from_start=rospy.Duration(
                        secs=motion_step*self.time_step))
            )
            rospy.loginfo('Motion Step: {}, joint = {}'.format(
                motion_step+1,
                self.motion_plan[motion_step+1, 1:7].tolist()
            ))
        pos_goal.trajectory.header.stamp = rospy.Time.now()


        for motion_step in range(self.motion_plan_size-1):
            pos_goal2.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=self.motion_plan[motion_step+1, 0:1].tolist(),
                    time_from_start=rospy.Duration(
                        secs=motion_step*self.time_step))
            )
            rospy.loginfo('Motion Step: {}, turntable_joint = {}'.format(
                motion_step+1,
                self.motion_plan[motion_step+1, 0:1].tolist()
            ))
        
        pos_goal2.trajectory.header.stamp = rospy.Time.now()

        #pdb.set_trace()
        # Send action goal
        self.client2.send_goal(pos_goal2)
        self.client.send_goal(pos_goal)
        
        self.client2.wait_for_result(rospy.Duration.from_sec(20.0))
        self.client.wait_for_result(rospy.Duration.from_sec(20.0))
      
    def global_move(self, motion_step):
        pos_goal = FollowJointTrajectoryGoal()
        pos_goal.trajectory.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        
        pos_goal2 = FollowJointTrajectoryGoal()
        pos_goal2.trajectory.joint_names = [
            'turntable_revolve_joint'
        ]

    
        pos_goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=self.motion_plan[motion_step, 1:7].tolist(),
                time_from_start=rospy.Duration(
                    secs=motion_step*self.time_step))
        )
        rospy.loginfo('Motion Step: {}, joint = {}'.format(
            motion_step,
            self.motion_plan[1:7,motion_step].tolist()
        ))
        pos_goal.trajectory.header.stamp = rospy.Time.now()


    
        pos_goal2.trajectory.points.append(
            JointTrajectoryPoint(
                positions=self.motion_plan[motion_step, 0:1].tolist(),
                time_from_start=rospy.Duration(
                    secs=motion_step*self.time_step))
        )
        rospy.loginfo('Motion Step: {}, turntable_joint = {}'.format(
            motion_step+1,
            self.motion_plan[motion_step, 0:1].tolist()
        ))
        
        pos_goal2.trajectory.header.stamp = rospy.Time.now()

        #pdb.set_trace()
        # Send action goal
        self.client2.send_goal(pos_goal2)
        self.client.send_goal(pos_goal)
        
        self.client2.wait_for_result(rospy.Duration.from_sec(20.0))
        self.client.wait_for_result(rospy.Duration.from_sec(20.0))

        return  self.motion_plan[motion_step, 1:7]


    def sweep_scan(self, current_global_pos):
         # Set joint goals
        time_step = 0.5

        pos_goal = FollowJointTrajectoryGoal()
        pos_goal.trajectory.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        alpha = current_global_pos[4]
        sweep_range = np.deg2rad(12)
        sweep_angles = np.array([alpha,  alpha-sweep_range/2,  alpha-sweep_range, alpha, alpha+sweep_range/2 , alpha+sweep_range, alpha]) #alpha-sweep_range/2,  alpha-sweep_range, alpha,
        #sweep_angles = np.repeat(sweep_angles, 5)
        sweep_plan = current_global_pos.reshape(-1, 1)
        sweep_plan = np.repeat(sweep_plan, len(sweep_angles), axis=1)
        sweep_plan[4,:] = sweep_angles
        for motion_step in range(len(sweep_angles)):
            pos_goal.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=sweep_plan[:,motion_step].tolist(),
                    time_from_start=rospy.Duration(
                        secs=motion_step*time_step))
            )
            rospy.loginfo('Motion Step: {}, joint = {}'.format(
                motion_step+1,
                sweep_plan[:,motion_step].tolist(),
            ))
        pos_goal.trajectory.header.stamp = rospy.Time.now()


        #pdb.set_trace()
        # Send action goal
     
        self.client.send_goal(pos_goal)
        
        
        self.client.wait_for_result(rospy.Duration.from_sec(20.0))

def move_manipulator():

    rospy.init_node('manipulator_joint_input')
    # pdb.set_trace()
    # Initialize kogumi base controller (time_step default is 2)
    arm_controller = ArmJointController(time_step=2.0)

    # Move Arm to initial position
    arm_controller.move_to_starting_pose()

    raw_input('Arm at the starting position. Press enter to continue.')

    # Move Arm
    for i in range(1, arm_controller.motion_plan_size):

        current_global_pos = arm_controller.global_move(i)
        # if i !=4:
        #  arm_controller.sweep_scan(current_global_pos)
        raw_input('Press enter to continue.')
        #pass

    # arm_controller.move_arm()


if __name__=='__main__':
    try:
        move_manipulator()
    except rospy.ROSInterruptException:
        pass