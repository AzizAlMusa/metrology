#!/usr/bin/env python

import sys
import rospy
from pcl_sensor_data.srv import *
# from avgZDistance.srv import *
# from laserMax.srv import *

def return_angle_client(status):
    rospy.wait_for_service('return_angle')
    try:
        return_angle = rospy.ServiceProxy('return_angle', angleAdjust)
        resp1 = return_angle(status)
        return resp1.angle
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e

def return_laser_client(status):
    rospy.wait_for_service('return_laser')
    try:
        return_laser = rospy.ServiceProxy('return_laser', laserMax)
        resp1 = return_laser(status)
        return resp1.laserMax
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e

def return_distance_client(status):
    rospy.wait_for_service('return_distance')
    try:
        return_distance = rospy.ServiceProxy('return_distance', avgZDistance)
        resp1 = return_distance(status)
        return resp1.distance
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e

def usage():
    return "%s [x y]"%sys.argv[0]


print("angle %d"  %return_angle_client(True))
print(return_laser_client(True))
print("distance %e" %return_distance_client(True))