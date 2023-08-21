#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import  Pose, Point, Quaternion, PoseArray
import trajectory_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


from visualization_msgs.msg import Marker

import tf.transformations as tr
import numpy as np


import pdb



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



class RobotJointSolver(object):

  def __init__(self):
    super(RobotJointSolver, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  
    ## This interface can be used to plan and execute motions:
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # table_name = "turntable"
    # table_group = moveit_commander.MoveGroupCommander(table_name)
    #move_group.set_goal_orientation_tolerance(np.deg2rad(45))

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    #Create a publisher for visualizing the path
    plot_trajectory_publisher = rospy.Publisher('plot_trajectory',
                                                   geometry_msgs.msg.PoseArray,
                                                   queue_size=20)

    controller_data_publisher = rospy.Publisher('controller_trajectory_data',
                                                   moveit_msgs.msg.RobotTrajectory,
                                                   queue_size=20)


   
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    # self.table_group = table_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.plot_trajectory_publisher = plot_trajectory_publisher
    self.controller_data_publisher = controller_data_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
  
  #converts pose from list [x, y, z, orientiation_x, orientation_y, orientation_z, orientation_w]
  #into a geometry_msgs.msg.Pose()
  def get_pose(self, raw_pose):
    pose = geometry_msgs.msg.Pose()

    pose.position.x = raw_pose[0]
    pose.position.y = raw_pose[1]
    pose.position.z = raw_pose[2]


    pose.orientation.x = raw_pose[3]
    pose.orientation.y = raw_pose[4]
    pose.orientation.z = raw_pose[5]
    pose.orientation.w = raw_pose[6]

    return pose
   
  def get_pose_array(self, raw_poses):
    pose_array = geometry_msgs.msg.PoseArray()
    for raw_pose in raw_poses:
        pose_array.poses.append(self.get_pose(raw_pose))

    return pose_array



  def go_to_joint_state(self, joint_goal = None):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the joint values from the group and adjust some of the values:
    if joint_goal is None:
         joint_goal = move_group.get_current_joint_values()
   
    if len(joint_goal) != len(move_group.get_current_joint_values()):
        print("Provided joints goal are not the same number of joints on robot")
        return

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()


    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)





  def go_to_pose_goal(self, pose_goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return plan, all_close(pose_goal, current_pose, 0.01)
   




  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

   
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

   


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
 


  
  #show as markers
  def show_path(self, waypoints):

        plan = waypoints
        self.plot_trajectory_publisher.publish(plan)
          
  #might be useful later?
  def check_singularity(self, plan):
   
    move_group = self.move_group

    joint_vals = move_group.get_current_joint_values()
    jac = move_group.get_jacobian_matrix(joint_vals)
    _, s, _ = np.linalg.svd(jac)
    manipulability = np.sqrt(np.linalg.det(np.matmul(jac,jac.T)))
    for point in plan.joint_trajectory.points:
      #pdb.set_trace()
      jac = move_group.get_jacobian_matrix(list(point.positions))
      manipulability = np.sqrt(np.linalg.det(np.matmul(jac,jac.T)))
      print(manipulability)


    return manipulability

  #for testing specific paths
  def create_test_plan(self):

    move_group = self.move_group

   

    pose_array = geometry_msgs.msg.PoseArray()
    poses_list = []
    n = 12
    for i in range(n):

        pose = move_group.get_random_pose().pose
  
        x = 0.1 * np.cos(2*np.pi / n * i)
        y = 0.1
        z = 0.1 * np.sin(2*np.pi / n * i) + 0.75

        translation = np.array([x, y, z])

        x_axis = np.array([0, 0, -1])
        y_axis = np.array([1, 0, 0])
        z_axis = np.array([0, -1, 0])

        pose = self.axis_to_pose(x_axis, y_axis, z_axis, translation)
        pose_array.poses.append(pose)
        poses_list.append(pose)
    

    pose = move_group.get_random_pose().pose

    x = 0
    y = 0.1
    z = 0.5 

    translation = np.array([x, y, z])

    x_axis = np.array([0, 0, -1])
    y_axis = np.array([1, 0, 0])
    z_axis = np.array([0, -1, 0])

    pose = self.axis_to_pose(x_axis, y_axis, z_axis, translation)
    pose_array.poses.append(pose)
    poses_list.append(pose)

    # new_pose = Pose(pose.position, pose.orientation)
    # angles = [30, -30, 0]
    # axes = [(0, 0, 1), (1, 0, 0)]

    # q1 = pose.orientation
    # q1 = np.array([q1.x, q1.y, q1.z, q1.w])
    # for axis in axes:
    #   for angle in angles:
       
    #     q2 = tr.quaternion_about_axis(np.deg2rad(angle), axis)

      
    #     q3 = tr.quaternion_multiply(q1,q2)
    #     X, Y, Z, W = q3
    #     new_pose.orientation = Quaternion(X, Y, Z, W)
    #     # pdb.set_trace()
    #     pose_array.poses.append(new_pose)
    #     poses_list.append(new_pose)
  
    
    return pose_array, poses_list

  
  def create_test_plan2(self):

    move_group = self.move_group

   

    pose_array = geometry_msgs.msg.PoseArray()
    poses_list = []
    num = 12
    steps = np.linspace(0, 0.5, 12)

    for step in steps:

        pose = move_group.get_random_pose().pose
  
        x = (-0.25) + step
        y = 0.25
        z = 0.5

        translation = np.array([x, y, z])
       
        x_axis = np.array([0, 0, -1])
        y_axis = np.array([1, 0, 0])
        z_axis = np.array([0, -1, 0])

        pose = self.axis_to_pose(x_axis, y_axis, z_axis, translation)
        pose_array.poses.append(pose)
        poses_list.append(pose)

    # steps = np.linspace(0, -0.5, 12)
    # for step in steps:

    #     pose = move_group.get_random_pose().pose
  
    #     x = (0.25) + step
    #     y = 0.25
    #     z = 0.5

    #     translation = np.array([x, y, z])
       
    #     x_axis = np.array([0, 0, -1])
    #     y_axis = np.array([1, 0, 0])
    #     z_axis = np.array([0, -1, 0])

    #     pose = self.axis_to_pose(x_axis, y_axis, z_axis, translation)
    #     pose_array.poses.append(pose)
    #     poses_list.append(pose)

    
    return pose_array, poses_list

  #convert axes and position vectors to a ros Pose
  def axis_to_pose(self, x_axis, y_axis, z_axis, translation):


        T = tr.rotation_matrix(0, np.array([0, 0, 1]))

        T[:3,0] = x_axis
        T[:3,1] = y_axis
        T[:3,2] = z_axis
        T[:3,3] = translation

        pos  = tr.translation_from_matrix(T)
        quat = tr.quaternion_from_matrix(T)
   
        frame_pose = Pose(Point(pos[0], pos[1], pos[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return frame_pose

  #the vcc segments viewpoints output only the centroids and normals
  #so the other 2 axes of the orientation are missing
  #this makes those axes with the logic of keep the EF upright
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


  #utility to read viewpoints of VCC segments
  def read_viewpoints(self):
        #pdb.set_trace()
        raw_read = np.loadtxt("/home/abdulaziz/pcl_ws/viewpoints.txt", delimiter=",", dtype="str")
        viewpoints = raw_read[:,:-1] #remove trailing comma ","
        viewpoints = viewpoints.astype(float)
        
        return viewpoints

  def read_viewpoints_tsp(self):
      #pdb.set_trace()
      raw_read = np.loadtxt("/home/abdulaziz/development/tsp/solution.csv", delimiter=",", dtype="str")
     
      viewpoints = raw_read.astype(float)
      
      return viewpoints

  #this is for x,y,z, x axis x, x axis y, axis z, y axis x, y axis y, y axis z, ....
  def read_raw_points(self):
      #pdb.set_trace()
      raw_read = np.loadtxt("/home/abdulaziz/pcl_ws/raw_poses.csv", delimiter=",", dtype="str")
      viewpoints = raw_read[:,:-1] #remove trailing comma ","
      # pdb.set_trace()
      viewpoints = viewpoints.astype(float)
      
      return viewpoints

  #reads the VCC segments info and constructs pose targets for the scanner
  def get_viewpoints(self):

        pose_array = PoseArray()
        poses_list = []
        #viewpoints = self.read_viewpoints_tsp() #self.read_viewpoints() #HERE MODIFY FOR TSP/RAW
        viewpoints = self.read_viewpoints()
        for viewpoint in viewpoints:
            #pdb.set_trace()
            z_axis = viewpoint[3:]
            x_axis, y_axis, z_axis = self.align_axis(z_axis)

            translation = viewpoint[:3] 
            translation[2] += 0.32
            frame_pose = self.axis_to_pose(x_axis, y_axis, z_axis, translation)

            if translation[2] > 0.30:
                pose_array.poses.append(frame_pose)
                poses_list.append(frame_pose)
        
        return pose_array, poses_list
    
    #reads the VCC segments info and constructs pose targets for the scanner
  def get_raw_points(self):

        pose_array = PoseArray()
        poses_list = []
        #viewpoints = self.read_viewpoints_tsp() #self.read_viewpoints() #HERE MODIFY FOR TSP/RAW
        viewpoints = self.read_raw_points()
        for viewpoint in viewpoints:
            # pdb.set_trace()

            x_axis = viewpoint[3:6]
            y_axis = viewpoint[6:9]
            z_axis = viewpoint[9:]

            x_axis /= np.linalg.norm(x_axis)
            y_axis /= np.linalg.norm(y_axis)
            z_axis /= np.linalg.norm(z_axis)

            translation = viewpoint[:3] 
            translation[2] += 0.32
            frame_pose = self.axis_to_pose(x_axis, y_axis, z_axis, translation)

            if translation[2] > 0.30:
                pose_array.poses.append(frame_pose)
                poses_list.append(frame_pose)
        
        return pose_array, poses_list
  

  def make_pose_csv(self, poses_list):
     
     output = np.zeros((len(poses_list), 7))
     for i, pose in enumerate(poses_list):

        current_pose = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        output[i, :] = np.array(current_pose)
     pdb.set_trace()
     np.savetxt("raw_poses.csv", output, delimiter=",")



  #simple get plan of a pose from current pose
  def get_joint_angles(self, pose):

    move_group = self.move_group

    plan = move_group.plan(pose)

    return plan


  #creates positions-only solution for writing to csv for the manual controller
  #this is not a good solution, pose goals are not connected, but are calculated from first pose each
  def get_total_plan(self, pose_list):
    plan = []
    for pose in pose_list:
        sub_plan = self.get_joint_angles(pose)
        #pdb.set_trace()
        for point in sub_plan.joint_trajectory.points:
            current_positions = list(point.positions)
            current_positions.insert(1, 0)
            current_positions[0] *= -1
            plan.append(current_positions)
    
    return plan



  #This gets the plan for multiple poses and executes them
  #Note it does not connect the paths so it simply keeps calculating from the first pose to each pose
  def execute_sub_plans(self, pose_list):
    
    for pose in pose_list:
        sub_plan = self.get_joint_angles(pose)
        
        self.execute_plan(sub_plan)
        #TODO Here we can insert a scanning routine





  def send_plan_to_controller(self, pose_list):
    
    self.controller_data_publisher.publish(pose_list)


  #need one more try to see if we can execute both arm and turntable from here
  #its better the manually sending to controller
  def execute_true_robot(self, pose_list):

    for pose in pose_list:

        sub_plan = self.get_joint_angles(pose)
        sub_plan.joint_trajectory.joint_names[0] = "turntable_revolve_joint"

        for point in sub_plan.joint_trajectory.points:
          
            positions = list(point.positions)
            positions[0] *= -1
            point.positions = positions

            velocities = list(point.velocities)
            velocities[0] *= -1
            point.velocities = velocities

            accelerations = list(point.accelerations)
            accelerations[0] *= -1
            point.accelerations = accelerations
    
            
        
        self.execute_plan(sub_plan)
    
  #this writes joint angle waypoints in a csv
  #[turntable, joint_0, joint_1, ...]
  def plan_to_csv(self, plan):
        import csv
        with open('../config/new_waypoints.csv', 'w') as file:
            writer = csv.writer(file)
           
            for position in plan:
             
                writer.writerow(position)

  #simply use the interface compute_`cartesi`an path
  def cartesian_compute(self, poses_list):

    move_group = self.move_group

    #TODO worth to experiment with jump_threshold to get smoother motions
    #why  does this take forever to solve?
    (plan, fraction) = move_group.compute_cartesian_path(
                                       poses_list,   # waypoints to follow
                                       0.005,        # eef_step
                                       5.0)         # jump_threshold    

    return plan, fraction
  

  #this is create a table_plan equivalent joint_0 but opposite direction
  #this is the joint position, velocity, acceleration and timing
  def virtual_to_turntable(self, plan):

    table_plan = moveit_msgs.msg.RobotTrajectory()
    table_plan.joint_trajectory.header.frame_id = "world"
    table_plan.joint_trajectory.joint_names = ["turntable_revolve_joint"]


    for point in  table_plan.joint_trajectory.points:

        pos = [-point.positions[0]]
        vel = [-point.velocities[0]]
        acc = [-point.accelerations[0]]

        secs = point.time_from_start.secs
        nsecs = point.time_from_start.nsecs

        table_plan.joint_trajectory.points

        table_point = trajectory_msgs.msg.JointTrajectoryPoint(
            positions=pos,
            velocities=vel,
            accelerations=acc,
            time_from_start=rospy.Duration(
                    secs=secs,
                    nsecs=nsecs)
        )

        table_plan.joint_trajectory.points.append(table_point)
        

  def sweep(self):

      move_group = self.move_group
      joint_goal = move_group.get_current_joint_values()

      joint_goal[-2] += joint_goal[-2] + np.deg2rad(10)
      
      move_group.go(joint_goal, wait=True)

      joint_goal[-2] += joint_goal[-2] + np.deg2rad(-10)

      move_group.go(joint_goal, wait=True)



def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Robot solver is running!"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
   

    solver = RobotJointSolver()

    #plan, poses_list = solver.create_test_plan()
    #solver.show_path(plan)
    #sub_plan = solver.get_joint_angles(poses_list[0])
    #solver.sweep()

    # pose_array, poses_list = solver.get_viewpoints()
    pose_array, poses_list = solver.create_test_plan()
    # solver.make_pose_csv(poses_list)
    

    # pose_array, poses_list = solver.get_raw_points()

    solver.show_path(pose_array)

    pdb.set_trace()
    # plan, frac = solver.cartesian_compute(poses_list)
    # print("the amount of trajectory found = " + str(frac))
   # print(frac)
    # pdb.set_trace()
    # for pose in poses_list:
      #pdb.set_trace()
      # plan = solver.get_joint_angles(pose)
      # solver.execute_plan(plan)
      # solver.sweep()
      # solver.send_plan_to_controller(plan)
      # pdb.set_trace()
    
    #pdb.set_trace()
    #pdb.set_trace()
    solution, frac = solver.cartesian_compute(poses_list)
    solver.execute_plan(solution)
    #out of box solution (moveit)
    #solver.execute_sub_plans(poses_list)

    #action client solutions
    # plan = solver.get_total_plan(poses_list)
    # solver.plan_to_csv(plan)

    #new action client publisher solution
    #solver.send_plan_to_controller(plan)
    print("done!")
   
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

