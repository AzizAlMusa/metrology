#!/usr/bin/env python

import Tkinter as tk
import rospy
from std_srvs.srv import Empty  # Replace with the service type you're using
from depth_processing.srv import CaptureMeasurement, GetMeasurement, AlignPointClouds, PublishAllMeasurements, GetIntersectionCloud, GenerateMesh, DeviationHeatmap, LoadMesh
# Initialize ROS node
rospy.init_node('depth_gui')

# Function to call a ROS service
def call_service(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, service_type)
        request = service_type._request_class()  # Initialize the request object
        # Populate request fields here if needed
        response = service(request)
        print("Service call to {} was successful.".format(service_name))
    except rospy.ServiceException as e:
        print("Service call to {} failed: {}".format(service_name, e))


root = tk.Tk()
root.title("ROS Service GUI")

button0 = tk.Button(root, text="Load Reference", command=lambda: call_service('/depth_processor_cpp/load_mesh', LoadMesh))
button0.pack()


button1 = tk.Button(root, text="Align Measurements", command=lambda: call_service('/depth_processor_cpp/align_measurements', AlignPointClouds))
button1.pack()

button2 = tk.Button(root, text="Capture Measurement", command=lambda: call_service('/depth_processor_cpp/capture_measurement', CaptureMeasurement))
button2.pack()

button3 = tk.Button(root, text="Get Measurement", command=lambda: call_service('/depth_processor_cpp/get_measurement', GetMeasurement))
button3.pack()

button4 = tk.Button(root, text="Publish All Measurements", command=lambda: call_service('/depth_processor_cpp/publish_all_measurements', PublishAllMeasurements))
button4.pack()

button5 = tk.Button(root, text="Get Intersection", command=lambda: call_service('/depth_processor_cpp/intersect_point_clouds', GetIntersectionCloud))
button5.pack()

button6 = tk.Button(root, text="Generate Mesh", command=lambda: call_service('/depth_processor_cpp/generate_mesh', GenerateMesh))
button6.pack()

button7 = tk.Button(root, text="Get Deviation Map", command=lambda: call_service('/depth_processor_cpp/deviation_mesh', DeviationHeatmap))
button7.pack()

button8 = tk.Button(root, text="Visualize Cloud", command=lambda: call_service('/depth_processor_cpp/visualize_cloud', DeviationHeatmap))
button8.pack()

root.mainloop()

