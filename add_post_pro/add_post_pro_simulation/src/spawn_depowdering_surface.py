#!/usr/bin/env python

import rospy, tf, os, rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from add_post_pro_simulation.srv import *
from gazebo_ros_link_attacher.srv import *

delete_model = None
spawn_model = None
height = .078       # Height of the scanning surface
rospack = None
path = None
surface_xml = None
box_xml = None
bunny_xml = None

    # pose.position.z = -0.0305;
    # pose.position.y = 0.635;
    #   <arg name="x" default="-0.01"/>
    #   <arg name="y" default="0"/>
    #   <arg name="z" default="0.0925"/>

def change_height(req):
    global delete_model, spawn_model, path, surface_xml

    item_name = "sample_surface"
    delete_model("sample_surface")
    rospy.sleep(.01)

    orient = tf.transformations.quaternion_from_euler(0,0,0)
    
    print("Spawning model")
    item_pose = Pose(position=Point(x=0, y=0.635,z=height+req.z_delta), orientation=Quaternion(0,0,0,0))
    spawn_model(item_name, surface_xml, "", item_pose, "world")
    return SurfaceHeightResponse(True)
    
def main():
    
    global delete_model, spawn_model, rospack, path, surface_xml
    rospy.init_node("spawn_surface")
        
    # Set up services
    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Services available")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    
    # Delete any models we wish to respawn
    delete_model("sample_surface")
    print("Cleared old models")
    rospy.sleep(1)

    # Find depowdering gazebo_models
    rospack = rospkg.RosPack()
    path = rospack.get_path('add_post_pro_simulation')

    # Load the surface model
    with open(path + "/gazebo_models/box_surface/model.sdf","r") as f:
        surface_xml = f.read()
        print("Got the model")

    surface_srv = rospy.Service('add_post_pro_simulation/change_height', SurfaceHeight, change_height)
    
    #initialize surface by calling service we just declared
    init_surface = rospy.ServiceProxy('add_post_pro_simulation/change_height', SurfaceHeight)
    init_surface(0)
    while not rospy.is_shutdown():
        rospy.spin()
if __name__ == '__main__':
    main()
