#!/usr/bin/env python
import os
from shutil import copyfile

import rospy
import rospkg

from add_post_pro_simulation.srv import *
path = None
rospack = None


def create_sdf_file(sdf, mesh_name_ext):
    mesh_name = mesh_name_ext.split(".")[0]
    sdf.write("""<?xml version='1.0'?>
    <sdf version='1.4'>@
    <model name=""" + mesh_name + """>
        <pose>0 0 0 0 0 0</pose>
        <!-- <static>1</static> -->
        <!-- <pose>0 0 0 0 0 0</pose> -->
        <link name=""" + mesh_name + """>
        <pose>0 0 0 0 0 0</pose>

        <inertial>
            <mass>1</mass>
            <inertia>
                <ixx>0.001</ixx>
                <ixy>0.0</ixy>
                <ixz>0.0</ixz>
                <iyy>0.001</iyy>
                <iyz>0.0</iyz>
                <izz>0.001</izz>
            </inertia>
        </inertial>

        <visual name="visual">
            <geometry>
            <mesh>
            <uri>model://""" + mesh_name + """/mesh/""" + mesh_name_ext + """</uri>
            <scale>0.001 0.001 0.001</scale>
            </mesh>
            </geometry>
        </visual>

        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>

        <collision name="collision">
            <geometry>
            <mesh>
            <uri>model://""" + mesh_name + """/mesh/""" + mesh_name_ext + """</uri>
            <scale>0.001 0.001 0.001</scale>
            </mesh>
            </geometry>
            <surface>
            <friction>
            <ode>
                <mu>2</mu>
                <mu2>2</mu2>
            </ode>
            <torsional>
                <ode/>
            </torsional>
            </friction>
            <bounce/>
            </surface>
            <max_contacts>10</max_contacts>
        </collision>
        </link>
  </model>
</sdf>
""")
    return True


def create_config_file(config, mesh_name):

    config.write("""<?xml version="1.0"?>
    <model>
        <name>""" + mesh_name + """</name>
        <version>1.0</version>
        <sdf version='1.4'>model.sdf</sdf>

        <author>
            <name> AddPostPro Member </name>
            <email> add_post_pro_member@andrew.cmu.edu</email>
        </author>

        <description>
            Auto-generated gazebo model.
        </description>
    </model>""")

    return True


def conversion_service(req):
    global path, rospack
    file_name = req.name
    mesh = None

    # Check if what we were given is a path or a file name
    name_split = file_name.split("/")
    mesh_name = name_split[len(name_split)-1]
    mesh_name_no_ext = 0    # Refers to the mesh without the extension
    file_name = 0   # File name should have the extension attached to it

    # Now we obtain the file name and mesh name
    if (len(name_split)==1):
        # No path provided
        check_ext = mesh_name.split(".")

        if len(check_ext)==1:   # No extension provided
            print("No explicit extension, assuming '.stl'")
            file_name = mesh_name +".stl"
            mesh_name_no_ext = mesh_name

        else:   # Extension provided
            print("Mesh name provided includes extension: " + check_ext[1])
            file_name = mesh_name
            mesh_name_no_ext = check_ext[0]


        print("Only model name is provided, looking in the add_post_pro_simulation meshes folder")
        mesh_path = path+"/mesh/"+file_name

        # TODO: Handle exceptions however they show up
    else:
        # For now, assume we are given a path beginning in the add_post_pro src library
        print("Attempting to navigate path to STL file")
        print("Assuming we are using a folder in the src library")
        overall_path = path+"/../"
        # name_split[0] is presumed to be the folder located in add_post_pro/src
        if not os.path.isdir(overall_path+name_split[0]):   # TODO: I don't think this functionality works
            print("Directory not located in project/src folder, returning")
            return
        else:
            mesh_path = overall_path+file_name+".stl"

    # Now in the gazebo_models folder, we will create the gazebo_model itself

    os.chdir(path+"/gazebo_models/")
    if os.path.isdir(path+"/gazebo_models/"+mesh_name_no_ext):
        print("Directory for this model already exists. Please delete it or rename it if you wish to remake the directory.")
        return ConversionRequestResponse(False)
    else:
        os.mkdir(path+"/gazebo_models/"+mesh_name_no_ext)  # Make overall directory
        os.mkdir(path+"/gazebo_models/"+mesh_name_no_ext+"/mesh/") # Make mesh subdirectory
        print("Directory created. Creating files in directory.")
        os.chdir(path+"/gazebo_models/"+mesh_name_no_ext) # Change to new model directory

        # Create model.config and model.sdf
        config = open("model.config", "a")
        sdf = open("model.sdf", "a")

        # Copy over mesh file
        print(mesh_path)
        copyfile(mesh_path, os.getcwd()+"/mesh/"+file_name)

        if (create_config_file(config, mesh_name_no_ext) and create_sdf_file(sdf, file_name)):
            return ConversionRequestResponse(True)
        else:
            return ConversionRequestResponse(False)

def main():
    global path, rospack
    rospy.init_node("mesh_conversion")

    # Find metrology gazebo_models
    rospack = rospkg.RosPack()
    path = rospack.get_path('add_post_pro_simulation')


    surface_srv = rospy.Service('add_post_pro_simulation/mesh_conversion', ConversionRequest, conversion_service)
    rospy.spin()

if __name__ == '__main__':
    main()
