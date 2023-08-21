#!/usr/bin/env python

import os

import rospy
import rospkg

from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from add_post_pro_simulation.srv import *
from gazebo_ros_link_attacher.srv import *

delete_model = None     # Gazebo service
spawn_model = None      # Gazebo service
rospack = None          # Used to find package paths
path = None             # Path to add_post_pro_simulation
model_list = {}         # Used to store names of models added through APP services


def app_change_surface_height(req):
    global delete_model, spawn_model, path, surface_xml

    if not os.path.isdir(path + "/gazebo_models/box_surface"):
        print("Could not find powderbed surface model in " + path + "/gazebo_models/")
        return APPChangePowderResponse(False)  # type:ignore

    with open(path + "/gazebo_models/box_surface/model.sdf", "r") as f:
        surface_xml = f.read()

    surface_name = "powder_surface"
    delete_model(surface_name)
    rospy.sleep(.01)

    surface_pose = Pose(position=Point(x=0, y=0.635, z=0.07 + req.z_delta), orientation=Quaternion(0, 0, 0, 0))
    spawn_model(surface_name, surface_xml, "", surface_pose, "world")
    return APPChangeHeightResponse(True)


def app_spawn_cube(req):
    global spawn_model, model_list, rospack, path
    if not os.path.isdir(path + "/gazebo_models/sample_box"):
        print("Could not find box model in " + path + "/gazebo_models/")
        return APPSpawnCubeResponse(False)

    with open(path + "/gazebo_models/sample_box/model.sdf", "r") as f:
        box_xml = f.read()

    delete_model("sample_box")
    rospy.sleep(.01)
    box_pose = Pose(position=Point(x=0, y=0.635, z=0.05))
    spawn_model("sample_box", box_xml, "", box_pose, "world")
    return APPSpawnCubeResponse(True)


def app_spawn_depowdering_env(req):
    global spawn_model, model_list, rospack, path

    # Grab box information from model sdf
    if not os.path.isdir(path + "/gazebo_models/product_box"):
        print("Could not find box model in " + path + "/gazebo_models/")
        return APPSpawnDepowderingResponse(False)

    with open(path + "/gazebo_models/product_box/model.sdf", "r") as f:
        box_xml = f.read()

    # Grab surface information from model sdf

    if not os.path.isdir(path + "/gazebo_models/box_surface"):
        print("Could not find powder surface model in " + path + "/gazebo_models/")
        return APPSpawnDepowderingResponse(False)

    with open(path + "/gazebo_models/box_surface/model.sdf", "r") as f:
        surface_xml = f.read()

    delete_model("product_box")  # Delete pre-existing versions of this model if necessary
    rospy.sleep(.01)
    box_pose = Pose(position=Point(x=-0.01, y=0.635, z=0.00), orientation=Quaternion(0, 0, 0, 0))
    spawn_model("product_box", box_xml, "", box_pose, "world")

    delete_model("powder_surface")
    rospy.sleep(.01)
    surface_pose = Pose(position=Point(x=0, y=0.635, z=0.07), orientation=Quaternion(0, 0, 0, 0))
    spawn_model("powder_surface", surface_xml, "", surface_pose, "world")

    return APPSpawnDepowderingResponse(True)


model_list = {}         # Used to store names of models added through add_post_pro services


def app_spawn_model(req):
    global spawn_model, model_list, rospack, path
    # Check if gazebo model exists
    if not os.path.isdir(path + "/gazebo_models/" + req.model_name):
        print("Could not find gazebo model in " + path + "/gazebo_models/")
        return APPSpawnModelResponse(False)

    # Gazebo model exists
    with open(path + "/gazebo_models/" + req.model_name + "/model.sdf", "r") as f:
        model_xml = f.read()
        print("Successfully loaded ", req.model_name)

    instance_name = req.instance_name
    pose = None
    parent_frame = None

    if not req.turntable:
        pose = req.pose
        parent_frame = "turntable_plate"
    else:
        pose = Pose(position=Point(x=0, y=0, z=0.005), orientation=Quaternion(0, 0, 0, 0))
        parent_frame = req.parent_frame
    if req.model_name in model_list:
        if req.instance_name in model_list[req.model_name]:
            print("Cannot add model as an instance of this model with the same instance name already exists.")
            return APPSpawnModelResponse(False)
        model_list[req.model_name].append(req.instance_name)
    else:
        model_list[req.model_name] = []
        model_list[req.model_name].append(instance_name)
    print(spawn_model(req.instance_name, model_xml, "", pose, parent_frame))

    model_1 = "robot"
    link_1 = "turntable_plate"
    model_2 = req.instance_name
    link_2 = req.model_name

    rospy.wait_for_service('/link_attacher_node/attach')
    try:
        attach_link = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

        resp = attach_link(model_1, link_1, model_2, link_2)
        print("Links attached!")
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: {} with response code {}".format(e, resp)) #f"Service call failed: {e} with response code {resp}"
    return APPSpawnModelResponse(True)


def app_delete_model(req):
    global delete_model, model_list

    model_name = None
    instance_name = None

    model_found = False     # noqa; F841
    instance_found = False  # noqa; F841
    # for key, value in model_list:
    #     if req.instance_name in value:
    #         if instance_found:
    #             print("[WARNING] Found two spawned models with the same name.")
    #         model_name = key
    #         model_found = True
    #         instance_found = True

    delete_model(req.instance_name)
    model_list[model_name].remove(instance_name)
    print("Deleted model")
    return APPDeleteModelResponse(True)


def app_list_models(req):
    global model_list

    models = []
    models.append("Model Name - Instance Name")
    for key, value in model_list:
        for i in value:
            models.append(key + " - " + i)
    return APPListModelsResponse(models)


def main():
    global delete_model, spawn_model, rospack, path
    rospy.init_node("spawn_model")

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    print("Found Gazebo services, setting up proxies")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    print("Connected to Gazebo services")

    rospack = rospkg.RosPack()
    path = rospack.get_path("add_post_pro_simulation")

    spawn_model_srv = rospy.Service("add_post_pro_simulation/spawn_model", APPSpawnModel, app_spawn_model)  # noqa; F841
    delete_model_srv = rospy.Service("add_post_pro_simulation/delete_model", APPDeleteModel, app_delete_model)  # noqa; F841
    list_models_srv = rospy.Service("add_post_pro_simulation/list_models", APPListModels, app_list_models)  # noqa; F841
    print("Set up spawn, delete, and list services for custom models")
    spawn_depowdering_srv = rospy.Service("add_post_pro_simulation/spawn_depowdering", APPSpawnDepowdering, app_spawn_depowdering_env)  # noqa; F841
    change_height_srv = rospy.Service("add_post_pro_simulation/change_height", APPChangeHeight, app_change_surface_height)  # noqa; F841
    spawn_box_srv = rospy.Service("add_post_pro_simulation/spawn_box", APPSpawnCube, app_spawn_cube)    # noqa; F841
    print("Set up services for depowdering environment spawning and manipulation")

    rospy.spin()

    pass


if __name__ == '__main__':
    main()