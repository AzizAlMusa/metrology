
#define _USE_MATH_DEFINES

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include <typeinfo>

#include <tf/transform_datatypes.h>

#include <math.h>
#include <random>

void pose_printer(geometry_msgs::Pose pose){

    std::cout << "position:" << std::endl;
    std::cout << "\tx: " << pose.position.x << std::endl;
    std::cout << "\ty: " << pose.position.y << std::endl;
    std::cout << "\tz: " << pose.position.z << std::endl;

    std::cout << "orientation:" << std::endl;
    std::cout << "\tx: " << pose.orientation.x << std::endl;
    std::cout << "\ty: " << pose.orientation.y << std::endl;
    std::cout << "\tz: " << pose.orientation.z << std::endl;
    std::cout << "\tw: " << pose.orientation.w << std::endl;

}


void demo1(std::vector<geometry_msgs::Pose> &pose_vector, moveit_visual_tools::MoveItVisualTools &visual_tools){
  namespace rvt = rviz_visual_tools;

 Eigen::Matrix3d R;
R << 1, 0, 0,
    0, 0, 1,
    0, -1, 0;

  Eigen::Quaterniond q(R);

float x_center = 0.0;
float y_center = 0.5;
float z_center = 0.75; 
  for (double x = x_center-0.25; x <= x_center+0.25; x += 0.05) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y_center;
    pose.position.z = z_center;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_vector.push_back(pose);
    visual_tools.publishAxisLabeled(pose, "pt" + std::to_string(x), rvt::SMALL);


  }

  for (double z = z_center; z <= z_center + 0.25; z+= 0.05) {
    geometry_msgs::Pose pose;
    pose.position.x = x_center;
    pose.position.y = y_center;
    pose.position.z = z;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_vector.push_back(pose);
    visual_tools.publishAxisLabeled(pose, "pt" + std::to_string(z), rvt::SMALL);


  }

  for (double z = z_center + 0.25; z >= z_center - 0.25; z-= 0.05) {
    geometry_msgs::Pose pose;
    pose.position.x = x_center;
    pose.position.y = y_center;
    pose.position.z = z;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_vector.push_back(pose);
    visual_tools.publishAxisLabeled(pose, "pt" + std::to_string(z), rvt::SMALL);


  }

}


void demo2(std::vector<geometry_msgs::Pose> &pose_vector, moveit_visual_tools::MoveItVisualTools &visual_tools){
  namespace rvt = rviz_visual_tools;

  Eigen::Matrix3d R;
R << 1, 0, 0,
0, 0, 1,
0, -1, 0;

  Eigen::Quaterniond q(R);

 float x_center = 0.0;
float y_center = 0.5;
float z_center = 0.75; 

  int num_points = 16*3;
 
  float radius = 0.1;
  float angle = 0;
  float increment = 3* 2 * M_PI / num_points;

  for (int i=0; i < num_points; i++){

    float x = x_center + radius * std::cos(angle);
    float y = y_center ;
    float z = z_center + radius * std::sin(angle);

    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_vector.push_back(pose);
    visual_tools.publishAxisLabeled(pose, "pt" + std::to_string(x), rvt::SMALL);

    angle += increment;


  }
}

std::vector<double> get_random_seed(){

    // Define the ranges for each random number
    double range1_min = -2.96706, range1_max = 2.96706;
    double range2_min = -1.745329, range2_max = 2.356194;
    double range3_min = -2.076942, range3_max = 2.949606;
    double range4_min = -3.14159, range4_max = 3.14159;
    double range5_min = -2.094395, range5_max = 2.094395;
    double range6_min = -3.14159, range6_max = 3.14159;

     // create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());


    // Create distributions for each range
    std::uniform_real_distribution<> dist1(range1_min, range1_max);
    std::uniform_real_distribution<> dist2(range2_min, range2_max);
    std::uniform_real_distribution<> dist3(range3_min, range3_max);
    std::uniform_real_distribution<> dist4(range4_min, range4_max);
    std::uniform_real_distribution<> dist5(range5_min, range5_max);
    std::uniform_real_distribution<> dist6(range6_min, range6_max);

  /// Generate the random numbers and store them in a vector
    std::vector<double> random_numbers;
    random_numbers.push_back(dist1(gen));
    random_numbers.push_back(dist2(gen));
    random_numbers.push_back(dist3(gen));
    random_numbers.push_back(dist4(gen));
    random_numbers.push_back(dist5(gen));
    random_numbers.push_back(dist6(gen));


    return random_numbers;
}


bool checkIkValidity(robot_state::RobotState* robot_state, const robot_state::JointModelGroup* joint_group, const double* joint_group_variable_values){
        return true; // Depending on your desition scheme.
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Denso Arm Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));




  std::vector<geometry_msgs::Pose> pose_vector;

  // demo1(pose_vector, visual_tools);


  demo2(pose_vector, visual_tools);


  double pose_tol = 0.01;

  // Create a PathConstraints object to enforce the end effector's orientation
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = move_group.getEndEffectorLink();
  ocm.header.frame_id = "world";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Apply the path constraint to the planning scene
  moveit_msgs::Constraints path_constraints;
  path_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(path_constraints);


  // Set the start state
  move_group.setStartStateToCurrentState();

  
  // Generate the Cartesian path plan
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(pose_vector, pose_tol, 0.0, trajectory);
  
   // Execute the plan
  move_group.execute(trajectory);


  visual_tools.publishPath(pose_vector, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL);
  // visual_tools.publishTrajectoryPath(trajectory, true);
  visual_tools.trigger();

  //TEST 1
  // Set target pose
  // move_group.setPoseTarget(main_pose);

  // Get the current state of the robot
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Set the start state to the current state and plan to the target pose
  // move_group.setStartState(*current_state);
  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  // bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // // Set a different start state and plan to the target pose
  // current_state->setToRandomPositions();
  // move_group.setStartState(*current_state);
  // success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // // Set another different start state and plan to the target pose
  // current_state->setToRandomPositions();
  // move_group.setStartState(*current_state);
  // success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // // Set yet another different start state and plan to the target pose
  // current_state->setToRandomPositions();
  // move_group.setStartState(*current_state);
  // success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return 0;
}