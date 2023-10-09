#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool

def call_service():
    # Initialize the ROS node
    rospy.init_node('call_invert_x_service')

    # Wait for the service to become available
    rospy.wait_for_service('/scancontrol_driver/invert_x')

    try:
        # Create a service proxy
        service_proxy = rospy.ServiceProxy('/scancontrol_driver/invert_x', SetBool)

        # Create a request object
        request = SetBool()
        request.data = False  # Set the request data to False

        # Call the service
        response = service_proxy(request)

        if response.success:
            rospy.loginfo("Service call was successful")
        else:
            rospy.logerr("Service call failed with message: %s", response.message)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    try:
        # Call the service with parameter False
        call_service()
    except rospy.ROSInterruptException:
        pass