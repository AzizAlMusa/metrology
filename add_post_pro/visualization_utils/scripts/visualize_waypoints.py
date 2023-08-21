#!/usr/bin/env python


import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion, PoseArray
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Header, ColorRGBA, String
import rviz_tools_py as rviz_tools

import moveit_msgs.msg
import tf.transformations as tr

import pdb


class TrajectoryVisualizer(object):

    def __init__(self):
# Initialize the ROS Node
        rospy.init_node('visualize_stuff', anonymous=False, log_level=rospy.INFO, disable_signals=False)


        rospy.on_shutdown(self.cleanup_node)

        self.markers = rviz_tools.RvizMarkers('world', 'visualization_marker')
        self.plan = None
        self.frame_angle = 0
# Define exit handler
    def cleanup_node(self):
        print("Shutting down node")
        self.markers.deleteAllMarkers()


    def plot_point(self, plan):
    
        self.markers.deleteAllMarkers()
        point = plan.poses[0]
        
    

        scale = 0.1
        self.markers.publishArrow(point, 'green', scale, 120.0)

    def plot_trajectory(self, plan):
        self.plan = plan
        
        self.markers.deleteAllMarkers()
        points = []
        
        for point in plan.poses:
            #points.append(Point(point.position.x, point.position.y, point.position.z))
            self.markers.publishAxis(point, 0.05, 0.01) # path, color, diameter, lifetime

       
        # for i in range(len(plan.poses) - 1):
        #      self.markers.publishLine(plan.poses[i],plan.poses[i+1], 'white', 0.005)


        # self.markers.publishLine(plan.poses[-1], plan.poses[0], 'white', 0.005)

        # self.markers.publishPath(plan.poses, 'green', 0.01)s
        #diameter= 0.05
        #markers.publishSpheres(points, 'green', diameter, 120.0)
        
    def transform_trajectory(self, joint_state):
        
        if self.plan is None:
            return
        
       
        angle = joint_state.actual.positions[0]
        rotation_angle = self.frame_angle - angle
        
        R = tr.rotation_matrix(-rotation_angle, np.array([0, 0, 1]), np.array([0, 0,0]))
            #R = tr.identity_matrix()
        quat = tr.quaternion_from_matrix(R)

        for pose in self.plan.poses:
            
           


            position_vec = np.ones((4,1))
            position_vec[0, 0] = pose.position.x
            position_vec[1, 0]  = pose.position.y
            position_vec[2, 0]  = pose.position.z
            
            new_position = np.matmul(R, position_vec)
            
            pose.position.x = new_position[0, 0]
            pose.position.y = new_position[1, 0]
            pose.position.z = new_position[2, 0]
            
            quat0 = np.zeros(4)
            quat0[0] = pose.orientation.x
            quat0[1] = pose.orientation.y
            quat0[2] = pose.orientation.z
            quat0[3] = pose.orientation.w
            new_q = tr.quaternion_multiply(quat, quat0)
            
            pose.orientation.x = new_q[0]
            pose.orientation.y = new_q[1]
            pose.orientation.z = new_q[2]
            pose.orientation.w = new_q[3]

        if np.abs(self.frame_angle - angle) > 1e-03:
            #pdb.set_trace()
           
            self.plot_trajectory(self.plan)
        self.frame_angle = angle
    
    #given the 3 axes of a frame and its position
    #computes its pose and publishes its marker
    def axis_to_pose(self, x_axis, y_axis, z_axis, translation):
        
        pdb.set_trace()
        T = tr.rotation_matrix(0, np.array([0, 0, 1]))

        T[:3,0] = x_axis
        T[:3,1] = y_axis
        T[:3,2] = z_axis
        T[:3,3] = translation

        pos  = tr.translation_from_matrix(T)
        quat = tr.quaternion_from_matrix(T)
   
        frame_pose = Pose(Point(pos[0], pos[1], pos[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        #pdb.set_trace()
        self.markers.publishAxis(frame_pose, 0.05, 0.01)

        

    # def align_axis(self):
    #     self.markers.deleteAllMarkers()

    #     T = tr.rotation_matrix(0, np.array([0, 0, 1]))

    #     x_axis = np.array([0, 0, -1])
    #     y_axis = np.array([1, 0, 0])
    #     z_axis = np.array([0, -1, 0])

    #     translation = np.array([0, 0.5, 0.5])

    #     T[:3,0] = x_axis
    #     T[:3,1] = y_axis
    #     T[:3,2] = z_axis
    #     T[:3,3] = translation

    #     position = tr.translation_from_matrix(T)
    #     quat = tr.quaternion_from_matrix(T)
    #     pdb.set_trace()

    #     vec = Pose(Point(position[0], position[1], position[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        

    #     self.markers.publishAxis(vec, 0.05, 0.01)

    #given some z_axis, get the x_axis to point downwards in the shorts direction towards xy plane
    #this is to match the end effector's frame: z_axis pointing towards target, x_axis downwards
    #maintain right-handedness
    def align_axis(self, z_axis):
        
        #pdb.set_trace()

        #z_axis = np.array([0, -1, -1])
        z_axis = z_axis / np.linalg.norm(z_axis)

        horizontal_plane = np.array([0, 0, 1])

        if 1 - np.abs(np.dot(horizontal_plane, z_axis)) < 0.01:
            #TODO have a smarter way to assign this e.g. match current EF y-axis 
            vertical_plane = np.array([1, 0, 0]) 
        else:
            vertical_plane = np.cross(horizontal_plane, z_axis)
            vertical_plane = vertical_plane / np.linalg.norm(vertical_plane)



        x_axis = np.cross(z_axis, vertical_plane)
        x_axis = x_axis / np.linalg.norm(x_axis)

        if x_axis[2] > 0:
            x_axis = x_axis * -1

        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        return x_axis, y_axis, z_axis


    def read_viewpoints(self):
        #pdb.set_trace()
        raw_read = np.loadtxt("/home/abdulaziz/pcl_ws/viewpoints.txt", delimiter=",", dtype="str")
        viewpoints = raw_read[:,:-1]
        viewpoints = viewpoints.astype(float)
        
        return viewpoints
     
    def show_viewpoints(self, viewpoints):

        self.markers.deleteAllMarkers()

        for viewpoint in viewpoints:
            #pdb.set_trace()
            z_axis = viewpoint[3:]
            x_axis, y_axis, z_axis = self.align_axis(z_axis)

            translation = viewpoint[:3] 
            # translation[2] += 0.32
            self.axis_to_pose(x_axis, y_axis, z_axis, translation)

        

       

        print('hello')
     

def main():
    
    visualizer = TrajectoryVisualizer()

    #this subscriber simply shows the waypoints as pose axes
    rospy.Subscriber("plot_trajectory", PoseArray, visualizer.plot_trajectory)

    #this subscriber handles the movement of waypoints with turntable
    #TODO need to find a cleaner solution
    rospy.Subscriber("/add_post_pro_equiv/turntable_controller/state", JointTrajectoryControllerState, visualizer.transform_trajectory)
    print("Waiting for waypoints")
    

    rospy.spin()

if __name__ == '__main__':
  main()

