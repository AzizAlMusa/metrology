
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/kinematics_base/kinematics_base.h>


#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <typeinfo>


#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

std::vector<std::vector<float>> readCSV(const std::string &fileName) {
    std::vector<std::vector<float>> data;
    std::string line;

    std::ifstream file(fileName);
    while (std::getline(file, line)) {
        std::vector<float> values;
        std::stringstream lineStream(line);
        std::cout << line << std::endl;
        float value;
        while (lineStream >> value) {
            values.push_back(value);
            if (lineStream.peek() == ',') {
                lineStream.ignore();
            }
        }
        data.push_back(values);
    }
    file.close();

    
    return data;
}





int main(int argc, char** argv){

    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
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

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


     move_group_interface.setPlannerId("PRM");
     ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    // Visualization
    // ^^^^^^^^^^^^^
    
    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    

     //PART 2 /////////////////////////////////////////////////////////

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    

 


    std::string fileName = "/home/abdulaziz/addpost_ws_new/raw_poses.csv";
    std::vector<std::vector<float>> data = readCSV(fileName);
    

    std::vector<geometry_msgs::Pose> waypoints;

    for (int i = 0; i < data.size(); i++)
    {
       
        geometry_msgs::Pose viewpoint;

        viewpoint.position.x  = data[i][0];
        viewpoint.position.y  = data[i][1];
        viewpoint.position.z  = data[i][2];

        viewpoint.orientation.x =  data[i][3];
        viewpoint.orientation.y =  data[i][4];
        viewpoint.orientation.z =  data[i][5];
        viewpoint.orientation.w =  data[i][6];

        // if (i % 5 == 0) {
        waypoints.push_back(viewpoint);
        // }

       

        
        geometry_msgs::Pose scan_up;
        scan_up.position = viewpoint.position;

        tf::Quaternion rotation_quat;
        rotation_quat.setRPY(0, M_PI/24, 0);

        

        tf::Quaternion current_orientation(viewpoint.orientation.x, viewpoint.orientation.y,  viewpoint.orientation.z, viewpoint.orientation.w);

        current_orientation *= rotation_quat;

        scan_up.orientation.x = current_orientation.x();
        scan_up.orientation.y = current_orientation.y();
        scan_up.orientation.z = current_orientation.z();
        scan_up.orientation.w = current_orientation.w();

        waypoints.push_back(scan_up);



        geometry_msgs::Pose scan_down;
        scan_down.position = viewpoint.position;

        tf::Quaternion rotation_quat2;
        rotation_quat2.setRPY(0, -M_PI/24, 0);

        tf::Quaternion current_orientation2(viewpoint.orientation.x, viewpoint.orientation.y,  viewpoint.orientation.z, viewpoint.orientation.w);
       
        current_orientation2 *= rotation_quat2;

        scan_down.orientation.x = current_orientation2.x();
        scan_down.orientation.y = current_orientation2.y();
        scan_down.orientation.z = current_orientation2.z();
        scan_down.orientation.w = current_orientation2.w();

        waypoints.push_back(scan_down);

        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }



       // Get the IK solver instance for the planning group
    // kinematics::KinematicsBasePtr ik_solver = kinematics::KinematicsBase::getSolverInstance(PLANNING_GROUP);

    // geometry_msgs::Pose desired_pose = waypoints[0];

    // std::vector<double> seed_state (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // std::vector<double> solution;

    // const double timeout = 0.1;

    // bool success = ik_solver->searchPositionIK(desired_pose, seed_state, timeout, solution);
    // if(success)
    //     std::cout<< "The Solution = " << solution[0] << std::endl;
    // else
    // ROS_ERROR("Failed to find IK solution");
        // visual_tools.trigger();
    
    // for (int i =0; i < waypoints.size(); ++i){

    // move_group_interface.setPoseTarget(waypoints[i]);
    // visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

   
    // visual_tools.trigger();
    // visual_tools.prompt("move?");
    // move_group_interface.move();

    // }
    ////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < waypoints.size(); i++){

        move_group_interface.setPoseTarget(waypoints[i]);
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(0), rvt::SMALL);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    
        visual_tools.trigger();
        // visual_tools.prompt("move?");
        move_group_interface.move();


    }
    
    // Cartesian Paths
    //###############################################################################//
    // Get the current pose of the robot
    geometry_msgs::Pose current_pose = move_group_interface.getCurrentPose().pose;

    

    // double value = atof(argv[1]);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    ROS_INFO_NAMED("tutorial", "Solving path plan");
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan X (Cartesian path) (%.2f%% achieved)", fraction * 100.0);



    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("J6"),  joint_model_group, rvt::LIME_GREEN );
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press next to execute.");

    move_group_interface.execute(trajectory);
    //#############################################################################//



    return 0;
}








//    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//     Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//     text_pose.translation().z() = 1.0;
//     visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

//     // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//     visual_tools.trigger();

//     // Getting Basic Information
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // We can print the name of the reference frame for this robot.
//     ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

//     // We can also print the name of the end-effector link for this group.
//     ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

//     // We can get a list of all the groups in the robot:
//     ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//     std::copy(move_group_interface.getJointModelGroupNames().begin(),
//                 move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

//     // Start the demo
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^
//     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//     // .. _move_group_interface-planning-to-pose-goal:
//     //
//     // Planning to a Pose goal
//     // ^^^^^^^^^^^^^^^^^^^^^^^
//     // We can plan a motion for this group to a desired pose for the
//     // end-effector.
//     geometry_msgs::Pose target_pose1;
//     target_pose1.orientation.w = 1.0;
//     target_pose1.position.x = 0.28;
//     target_pose1.position.y = 0.25;
//     target_pose1.position.z = 0.8;
//     move_group_interface.setPoseTarget(target_pose1);

//     // Now, we call the planner to compute the plan and visualize it.
//     // Note that we are just planning, not asking move_group_interface
//     // to actually move the robot.
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//     bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//     ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//     // Visualizing plans
//     // ^^^^^^^^^^^^^^^^^
//     // We can also visualize the plan as a line with markers in RViz.
//     ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//     visual_tools.publishAxisLabeled(target_pose1, "pose1");
//     visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


//     // Moving to a pose goal
//     // ^^^^^^^^^^^^^^^^^^^^^
//     //
//     // If you do not want to inspect the planned trajectory,
//     // the following is a more robust combination of the two-step plan+execute pattern shown above
//     // and should be preferred. Note that the pose goal we had set earlier is still active,
//     // so the robot will try to move to that goal.

//     move_group_interface.move();


//     visual_tools.trigger();
    

    // // Cartesian Paths

    // // Get the current pose of the robot
    // geometry_msgs::Pose current_pose = move_group_interface.getCurrentPose().pose;

    // // Create three waypoints
    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(current_pose);

    // geometry_msgs::Pose target_pose3 = current_pose;

    // target_pose3.position.y -= 0.2;
    // waypoints.push_back(target_pose3);  

    // target_pose3.position.z += 0.2;
    // target_pose3.position.y += 0.2;
    // target_pose3.position.x -= 0.2;
    // waypoints.push_back(target_pose3);  

    
    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // // Visualize the plan in RViz
    // visual_tools.deleteAllMarkers();
    // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    // for (std::size_t i = 0; i < waypoints.size(); ++i)
    //     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // move_group_interface.execute(trajectory);

