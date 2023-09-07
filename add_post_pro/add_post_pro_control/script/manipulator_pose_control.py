#!/usr/bin/python

import time
from time import sleep
from os import path

import numpy as np

import rospy
import rospkg
import actionlib

import joint_angles_ikfast

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import openravepy as orpy
from openravepy import *

class ArmJointController:

    def __init__(self, time_step=0.1):

        # Set a publishing rate
        self.time_step = time_step
        self.rate = rospy.Rate(1/time_step)

        # Initialize action client to the denso arm
        self.client = actionlib.SimpleActionClient(
            "/add_post_pro_robot_b120/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self.client.wait_for_server()
        self.create_motion_plan()
        # self.load_motion_plan()

    def create_motion_plan(self):
        robot_file = '/home/cerlab/catkin_ws/src/add_post_pro2/add_post_pro_bringup/src/env.xml'
        pose_file = '/home/cerlab/catkin_ws/src/add_post_pro2/add_post_pro_bringup/src/poses-test.csv'

        solutions = joint_angles_ikfast.find_joint_angles(robot_file, pose_file)
        self.load_motion_plan(solutions)

    # def openrave_setup(self):
    #     env = orpy.Environment()
    #     env.Load('/home/cerlab/catkin_ws/src/add_post_pro2/add_post_pro_bringup/src/env.xml')
    #     # env.SetViewer('qtcoin')
    #     # viewer = env.GetViewer()
    #     # viewer.SetCamera([
    #     # [-0.84869234,  0.12028422, -0.5150272 ,  1.3087337 ],
    #     # [ 0.52819784,  0.2424563 , -0.81377023,  1.55376744],
    #     # [ 0.02698787, -0.96267681, -0.26930467,  0.95547998],
    #     # [ 0.        ,  0.        ,  0.        ,  1.        ]])
    #     # viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    #     robot = env.GetRobots()[0] #Import first robot
    #     table = env.GetRobots()[1]

    #     poses = np.genfromtxt("/home/cerlab/catkin_ws/src/add_post_pro2/add_post_pro_bringup/src/poses-test.csv",delimiter = ",")
    #     raveLogInfo("Pose List read. Current shape : {}".format(poses.shape))
    #     # -------------------IKFAST Implementation--------------------------------------
    #     robot.SetActiveManipulator('arm') # set the manipulator to leftarm + torso
    #     manip = robot.GetActiveManipulator()
    #     ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D, forceikfast = True)
    #     if not ikmodel.load():
    #         ikmodel.autogenerate()
    #     manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
    #     rospy.loginfo("IK Model Generated successfully. Current IK Name = {}".format(ikmodel.getikname()))
    #     # Tgoal1 = np.array([[1,0,0,0.5],[0,1,0,0.5],[0,0,1,0.5],[0,0,0,1]])
    #     # Tgoal2 = np.array([[1,0,0,0.5],[0,1,0,-0.5],[0,0,1,0.5],[0,0,0,1]])
    #     # goal_pose = [Tgoal1, Tgoal2]
        
        # joint_angles = robot.GetDOFValues()
        # print("List : {}".format(robot.GetDOFValues()))
        # for i in range(poses.shape[0]):
        #     Tgoal = poses[i,:].reshape((4,4))
        #     # print(Tgoal)
        #     start_time = time.time()
        #     res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
        #     plan_time = time.time()
        #     rospy.loginfo("Planning Time for step = {} seconds".format(str(round(plan_time - start_time,3))))
        #     robot.WaitForController(0) # wait
        #     end_time = time.time()
        #     rospy.loginfo("Execution Time for step = {} seconds".format(str(round(end_time - plan_time,3))))
        #     joint_angles = np.append(joint_angles, robot.GetDOFValues())
        
        #     print("List : {}".format(joint_angles))
        #     Tee = manip.GetEndEffectorTransform()
        #     point = Tee[0:3,3]
        #     h = env.plot3(point,10) # plot one point
        #     print("Next Step")
        # rospack = rospkg.RosPack()
        # print(joint_angles)
        # np.savetxt(path.join(rospack.get_path('add_post_pro_control'),
        #     'config',
        #     'manipulator_motion_plan_test.csv'),
        #     joint_angles.reshape(6,-1),
        #     delimiter = ",")


    # def load_motion_plan(self, solutions):
    #     # Get current package path
    #     rospack = rospkg.RosPack()
    #     motion_plan_file = path.join(
    #         rospack.get_path('add_post_pro_control'),
    #         'config',
    #         'manipulator_motion_plan_test.csv')
    #     rospy.loginfo('Load motion plan file: ' + motion_plan_file)

    #     # Load motion plan
    #     self.motion_plan = np.genfromtxt(motion_plan_file, delimiter=',')
    #     rospy.loginfo('Loaded motion plan size: ' + str(self.motion_plan.shape))
    #     self.motion_plan_size = self.motion_plan.shape[1]
    #     self.motion_step_count = 0

    def load_motion_plan(self, solutions):
        
        # Load motion plan
        self.motion_plan = solutions
        rospy.loginfo('Loaded motion plan size: ' + str(self.motion_plan.shape))
        self.motion_plan_size = self.motion_plan.shape[1]
        self.motion_step_count = 0


    def move_to_starting_pose(self):

        # Set joint goals
        time_step = 20
        
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

    # raw_input('Arm at the starting position. Press enter to continue.')

    # Move Arm
    arm_controller.move_arm()


if __name__=='__main__':
    start = time.time()
    try:
        move_manipulator()
    except rospy.ROSInterruptException:
        pass
    end = time.time()
    rospy.loginfo('Runtime = {} s'.format(end-start))