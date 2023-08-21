#!/usr/bin/env python

import sys
import rospy

from gazebo_ros_link_attacher.srv import *

def gazebo_ros_link_attacher_client(model_1, link_1, model_2, link_2):
    
    rospy.wait_for_service('/link_attacher_node/attach')

    try:
        attach_link = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

        resp = attach_link(model_1, link_1, model_2, link_2)

        return resp

    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s", e)


if __name__ == '__main__':

    rospy.init_node("link_attacher")
    
    ## Arguments works in command, but not in launch file
    ## 2019-08-01 00:52:30

    # if len(sys.argv) == 5:
    #     model_1 = sys.argv[1]
    #     link_1 = sys.argv[2]
    #     model_2 = sys.argv[3]
    #     link_2 = sys.argv[4]
    # else:
    #     rospy.logwarn("%s model_1 link_1 model_2 link_2", sys.argv[0])
    #     sys.exit(1)

    if rospy.has_param('~model_1'):
        model_1 = rospy.get_param('~model_1')
    if rospy.has_param('~link_1'):
        link_1 = rospy.get_param('~link_1')
    if rospy.has_param('~model_2'):
        model_2 = rospy.get_param('~model_2')
    if rospy.has_param('~link_2'):
        link_2 = rospy.get_param('~link_2')
    
    resp = gazebo_ros_link_attacher_client(model_1, link_1, model_2, link_2)


