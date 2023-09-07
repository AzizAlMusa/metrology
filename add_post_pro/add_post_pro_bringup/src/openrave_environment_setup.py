#!/usr/bin/env python

import array
from time import sleep
import time
import os
import sys,time
from types import ModuleType
from numpy import *
from optparse import OptionParser

import numpy as np

import rospy
# import rospkg
import actionlib

import openravepy as orpy
from openravepy import *


print("Import success")
if __name__ == "__main__":
    env = orpy.Environment()
    env.Load('env.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetCamera([
    [-0.84869234,  0.12028422, -0.5150272 ,  1.3087337 ],
    [ 0.52819784,  0.2424563 , -0.81377023,  1.55376744],
    [ 0.02698787, -0.96267681, -0.26930467,  0.95547998],
    [ 0.        ,  0.        ,  0.        ,  1.        ]])
    # viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobots()[0] #Import first robot
    table = env.GetRobots()[1] #Import second robot
    raveLogInfo("Setup Initialised")

    poses = np.genfromtxt("/home/cerlab/catkin_ws/src/add_post_pro2/add_post_pro_bringup/src/poses-test.csv",delimiter = ",")
    raveLogInfo("Pose List read. Current shape : {}".format(poses.shape))
     # -------------------IKFAST Implementation--------------------------------------
    robot.SetActiveManipulator('arm') # set the manipulator to leftarm + torso
    manip = robot.GetActiveManipulator()
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D, forceikfast = True)
    if not ikmodel.load():
        ikmodel.autogenerate()
    manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
    raveLogInfo("IK Model Generated successfully. Current IK Name = {}".format(ikmodel.getikname()))
    # Tgoal1 = np.array([[1,0,0,0.5],[0,1,0,0.5],[0,0,1,0.5],[0,0,0,1]])
    # Tgoal2 = np.array([[1,0,0,0.5],[0,1,0,-0.5],[0,0,1,0.5],[0,0,0,1]])
    # goal_pose = [Tgoal1, Tgoal2]
    
    for i in range(poses.shape[0]):
        Tgoal = poses[i,:].reshape((4,4))
        # print(Tgoal)
        start_time = time.time()
        res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
        plan_time = time.time()
        raveLogInfo("Planning Time for step = {} seconds".format(str(round(plan_time - start_time,3))))
        robot.WaitForController(0) # wait
        end_time = time.time()
        raveLogInfo("Execution Time for step = {} seconds".format(str(round(end_time - plan_time,3))))
        Tee = manip.GetEndEffectorTransform()
        point = Tee[0:3,3]
        h = env.plot3(point,10) # plot one point
        print("Next Step")
    
    # -------------------Transform 3D--------------------------------------
    # Works well. Too Many Solutions tho...
    # manip = robot.GetActiveManipulator()
    # ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
    # if not ikmodel.load():
    #     ikmodel.autogenerate()
    # raveLogInfo("Manipulator creation complete")
    # with robot: # lock environment and save robot state
    #     robot.SetDOFValues([2.58, 0.547, 1.5],[0,1,2]) # set the first 3 dof values
    #     Tee = manip.GetEndEffectorTransform() # get end effector
    #     ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    #     sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions
    # raveLogInfo("First save complete")
    # h = env.plot3(Tee[0:3,3],10) # plot one point
    # with robot: # save robot state
    #     i = 1
    #     raveLogInfo('%d solutions'%len(sols))
    #     for sol in sols: # go through every solution
    #         robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
    #         env.UpdatePublishedBodies() # allow viewer to update new robot
    #         time.sleep(5)
    #         raveLogInfo("Solution %d"%i)
    #         i+=1

    # raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original
    # --------------------Transfrom 6D------------------------------------
    # manip = robot.SetActiveManipulator('arm') # set the manipulator to leftarm
    # ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    # if not ikmodel.load():
    #     ikmodel.autogenerate()

    # # time.sleep(5)
    # with env: # lock environment
    #     # Tgoal = np.array([[0,-1,0,-0.21],[-1,0,0,0.04],[0,0,-1,0.92],[0,0,0,1]])
    #     Tgoal = np.array([[1,0,0,0.5],[0,1,0,0.5],[0,0,1,0.5],[0,0,0,1]])
    #     sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    #     raveLogInfo('Solution is: '+repr(sol))
    #     with robot: # save robot state
    #         robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
    #         Tee = manip.GetEndEffectorTransform()
    #         env.UpdatePublishedBodies() # allow viewer to update new robot
    #         time.sleep(20)
    
    #     raveLogInfo('Tee is: '+repr(Tee))

    # ----------------------Visual Debug---------------------------------------
    # RaveSetDebugLevel(DebugLevel.Debug)
    # manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
    # manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548]) # call motion planner with goal joint angles
    # robot.WaitForController(0) # wait
    # robot.createManipulator()
    # robot.SetActiveManipulator('J0_J6')
    # maniprob = interfaces.BaseManipulation(robot)
    # maniprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19])
    # robot.WaitForController(0)
    # -----------------------Setup--------------------------------------
    # with env: # lock the environment since robot will be used
        # orpy.raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))
        # robot.SetDOFValues([0,0.6,1.8,0,-0.8,0],[0,1,2,3,4,5]) # set joint 0 to value 0.5
        # T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
        # orpy.raveLogInfo("The transformation of link 1 is:\n"+repr(T))
        # orpy.raveLogInfo("Robot "+table.GetName()+" has "+repr(table.GetDOF())+" joints with values:\n"+repr(table.GetDOFValues()))
        # time.sleep(50)
        # Tz = orpy.matrixFromAxisAngle([0,0,0])
        # for body in env.GetBodies():
        #     body.SetTransform(np.dot(Tz, body.GetTransform()))
       