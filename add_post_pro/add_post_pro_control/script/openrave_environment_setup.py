#!/usr/bin/python

from time import sleep
import os

import numpy as np

import rospy
import rospkg
import actionlib

# Node Initialisation
rospy.init_node('openrave_launch')

# Finding Descirption files
rospack = rospkg.RosPack()
robot_description_path = os.path.join(rospack.get_path('denso_robot_ros'),
            'denso_robot_description',
            'vs6577_description',
            'urdf',
            'vs6577.urdf.xacro')
rospy.loginfo('Load Description File: ' + robot_description_path)
robot_name = "vs6577"

# Changing Directory to the path found
dir_change = os.system("cd {}".format(robot_description_path))
print("Directory change ran with exit code %d" % dir_change)

# Converting .urdf.xacro > .urdf > .dae
cvt_urdf = os.system("rosrun xacro xacro --inorder -o /tmp/{}.urdf {}.urdf.xacro".format(robot_name, robot_name))
print("Conversion to urdf ran with exit code %d" % cvt_urdf)

x = os.system("cd tmp")

cvt_dae = os.system("rosrun collada_urdf urdf_to_collada {}.urdf {}.dae".format(robot_name, robot_name))
print("Conversion to dae ran with exit code %d" % cvt_dae)

# Calling openrave to open file
x = os.system("openrave {}.dae".format(robot_name))