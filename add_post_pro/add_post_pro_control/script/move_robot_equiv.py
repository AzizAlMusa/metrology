#!/usr/bin/python

from time import sleep
from os import path

import numpy as np

import rospy
import rospkg
import actionlib

import pdb

from moveit_msgs.msg import RobotTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class ArmJointController:

    def __init__(self, time_step=0.1):


        #rospy.init_node('manual_controller', anonymous=False, log_level=rospy.INFO, disable_signals=False)
        #rospy.on_shutdown(self.cleanup_node)

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

    def cleanup_node(self):
        print("Shutting down node")

    def load_motion_plan(self):
        # Get current package path
        rospack = rospkg.RosPack()
        motion_plan_file = path.join(
            rospack.get_path('add_post_pro_control'),
            'config',
            'new_waypoints.csv')
        rospy.loginfo('Load motion plan file: ' + motion_plan_file)

        # Load motion plan
        self.motion_plan = np.genfromtxt(motion_plan_file, delimiter=',')


        rospy.loginfo('Loaded motion plan size: ' + str(self.motion_plan.shape))
        #pdb.set_trace()
        self.motion_plan_size = self.motion_plan.shape[0]
        self.motion_step_count = 0

    def move_as_plan(self, trajectory):

        self.send_to_controller(trajectory)
        self.send_to_controller2(trajectory)
    
    def send_to_controller(self, trajectory):
        
        pos_goal = FollowJointTrajectoryGoal()

        #pdb.set_trace()
        pos_goal.trajectory = trajectory.joint_trajectory

        self.client.send_goal(pos_goal)

        self.client.wait_for_result(rospy.Duration(5.0))

    def send_to_controller2(self, trajectory):
        pass

    def move_to_starting_pose(self):

        # Set joint goals
        time_step = 0.25
        pos_goal = FollowJointTrajectoryGoal()
        pos_goal.trajectory.joint_names = [
            'joint_0',
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
                positions=self.motion_plan[0,1:8].tolist(),
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
                self.motion_plan[0,1:8].tolist()
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
            'joint_0',
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
                    positions=self.motion_plan[motion_step+1, 1:8].tolist(),
                    time_from_start=rospy.Duration(
                        secs=motion_step*self.time_step))
            )
            rospy.loginfo('Motion Step: {}, joint = {}'.format(
                motion_step+1,
                self.motion_plan[motion_step+1, 1:8].tolist()
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
                'joint_0',
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
                    positions=self.motion_plan[motion_step, 1:8].tolist(),
                    time_from_start=rospy.Duration(
                        secs=motion_step*self.time_step))
            )
            rospy.loginfo('Motion Step: {}, joint = {}'.format(
                motion_step,
                self.motion_plan[motion_step, 1:8].tolist()
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

def move_manipulator():

    rospy.init_node('manipulator_joint_input')
    # pdb.set_trace()
    # Initialize kogumi base controller (time_step default is 2)
    arm_controller = ArmJointController(time_step=0.25)
    

    rospy.Subscriber("controller_trajectory_data", RobotTrajectory, arm_controller.move_as_plan)

    rospy.spin()
    # Move Arm t    o initial position
    #arm_controller.move_to_starting_pose()

    #raw_input('Arm at the starting position. Press enter to continue.')

   
    #for i in range(1, arm_controller.motion_plan_size):

        #current_global_pos = arm_controller.global_move(i)
        # if i !=4:
        #  arm_controller.sweep_scan(current_global_pos)
        #raw_input('Press enter to continue.')
        #pass

    #arm_controller.move_arm()


if __name__=='__main__':
    try:
        move_manipulator()
    except rospy.ROSInterruptException:
        pass