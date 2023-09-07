#!/usr/bin/python

from os import path
import rospy
import rospkg
import actionlib
import numpy as np
import openravepy
import csv
import tf

def find_joint_angles(robot_file = '/home/cerlab/catkin_ws/src/add_post_pro2/add_post_pro_bringup/src/env.xml', 
                      pose_file = '/home/cerlab/catkin_ws/src/add_post_pro2/add_post_pro_bringup/src/poses-test.csv'):

    # Load the robot model and initialize the inverse kinematics solver
    env = openravepy.Environment()
    env.Load(robot_file)

    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetCamera([
        [-0.84869234,  0.12028422, -0.5150272 ,  1.3087337 ],
        [ 0.52819784,  0.2424563 , -0.81377023,  1.55376744],
        [ 0.02698787, -0.96267681, -0.26930467,  0.95547998],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple

    robot = env.GetRobots()[0]
    manipulator = robot.SetActiveManipulator("arm")
    manipprob = openravepy.interfaces.BaseManipulation(robot)

    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=openravepy.IkParameterizationType.Transform6D, forceikfast = True)
    ikmodel.load()
    if not ikmodel.load():
        ikmodel.autogenerate()
    print("IKFast Initialised")

    # Read the CSV file with end effector poses
    poses = np.genfromtxt(pose_file, delimiter=',')
    print("Poses Initialised")
    # Solve the inverse kinematics for each pose and store the joint angles
    solutions = np.array([])
    for pose in poses:
        # print("Calculations for a pose started")
        # x, y, z, a, b, c = pose
        # print("Pose = {}".format([x,y,z,a,b,c]))
        # quat = tf.transformations.quaternion_from_euler(a, b, c)
        # matrix = np.eye(4)
        # matrix[:3, 3] = [x, y, z]
        # matrix[:3, :3] = tf.transformations.quaternion_matrix(quat)[:3, :3]
        
        # res = manipprob.MoveToHandPosition(matrices=[matrix],seedik=10)
        # robot.WaitForController(0)
        
        # print("Transform = \n {}".format(matrix))
        # solution = manipulator.FindIKSolution(matrix, openravepy.IkFilterOptions.CheckEnvCollisions)
        # print("Solution = {}".format(solution))
        # np.append(solutions,solution)
        print("Pose Matrix : {}".format(pose))
        Tgoal = pose.reshape((4,4))
        # print(Tgoal)
        res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
        robot.WaitForController(0) # wait
        solutions = np.append(solutions, robot.GetDOFValues())
        Tee = manipulator.GetEndEffectorTransform()
        point = Tee[0:3,3]
        h = env.plot3(point,10) # plot one point
        print("Solution = \n {}".format(solutions))
        print("Next Step")
    solutions = solutions.reshape((6,-1))
    print("Solutions ready")
    print(solutions)
    print("Complete")
    # return solutions
    rospack = rospkg.RosPack()
    joint_file = path.join(rospack.get_path('add_post_pro_control'),'config','manipulator_motion_plan_test.csv')

    # Write the solutions to another CSV file
    np.savetxt(joint_file, solutions, delimiter=',')
    print("Complete")


find_joint_angles()