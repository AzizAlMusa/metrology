#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

// Function to generate a straight line trajectory
std::vector<geometry_msgs::Pose> generateStraightLineTrajectory(const geometry_msgs::Pose& start_pose) {
    geometry_msgs::Pose end_pose = start_pose;
    end_pose.position.x += 0.5;  // Move 0.5 meters in the x direction

    std::vector<geometry_msgs::Pose> waypoints;
    for (double i = 0.01; i <= 1; i += 0.01) {  // Start from i=0.01 to skip the start_pose
        geometry_msgs::Pose waypoint = start_pose;
        waypoint.position.x += (end_pose.position.x - start_pose.position.x) * i;
        waypoints.push_back(waypoint);
    }

    return waypoints;
}

// Function to generate a helical trajectory
std::vector<geometry_msgs::Pose> generateHelicalTrajectory(const geometry_msgs::Pose& start_pose) {
    const double height = 0.25;  // Total height of the helix
    const double radius = 0.1;  // Radius of the helix
    const double turns = 2;     // Number of turns in the helix
    const double step_size = 0.01;  // Incremental step for generating waypoints

    std::vector<geometry_msgs::Pose> waypoints;
    for (double z = 0; z <= height; z += step_size) {
        double angle = turns * 2.0 * M_PI * (z / height);  // Calculate the angle based on height
        geometry_msgs::Pose waypoint = start_pose;

        waypoint.position.x += radius * cos(angle);  // x-coordinate based on the radius and angle
        waypoint.position.y += radius * sin(angle);  // y-coordinate based on the radius and angle
        waypoint.position.z += z;  // z-coordinate based on height

        waypoints.push_back(waypoint);
    }

    return waypoints;
}


// Function to generate a trajectory that mimics the orientation of drawing out the vertices of a cube
std::vector<geometry_msgs::Pose> generateCubeOrientationTrajectory(const geometry_msgs::Pose& start_pose) {
    std::vector<geometry_msgs::Pose> waypoints;

    // Define the rotation angles for the cube vertices in orientation space
    std::vector<double> angles = {M_PI/4, -M_PI/4};  // 45 degrees for a softer transition

    // For each axis combination
    for (double x_angle : angles) {
        for (double y_angle : angles) {
            for (double z_angle : angles) {
                geometry_msgs::Pose rotated_pose = start_pose;

                // Convert the current orientation to a quaternion
                tf::Quaternion q(start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);

                // Apply the rotations for the current combination of angles
                tf::Quaternion rotation = tf::createQuaternionFromRPY(x_angle, y_angle, z_angle);
                q *= rotation;

                rotated_pose.orientation.x = q.x();
                rotated_pose.orientation.y = q.y();
                rotated_pose.orientation.z = q.z();
                rotated_pose.orientation.w = q.w();

                waypoints.push_back(rotated_pose);
            }
        }
    }

    return waypoints;
}





// Main execution function to plan and execute a given trajectory
void executeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints, moveit::planning_interface::MoveGroupInterface& move_group_interface) {
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm");
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("J6"), joint_model_group, rvt::LIME_GREEN);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press next to execute.");

    move_group_interface.execute(trajectory);
}

// Function to generate a figure-eight (lemniscate) trajectory in the X-Y plane
std::vector<geometry_msgs::Pose> generateFigureEightTrajectory(const geometry_msgs::Pose& start_pose) {
    std::vector<geometry_msgs::Pose> waypoints;

    const double a = 0.2;  // Semi-major axis of the lemniscate, controls the size of the figure-eight
    const int num_points = 100;  // Number of waypoints to generate
    const double step = 2.0 * M_PI / num_points;  // Incremental step for generating waypoints

    for (double t = 0; t <= 2.0 * M_PI; t += step) {
        geometry_msgs::Pose waypoint = start_pose;

        // Lemniscate parametric equations
        waypoint.position.x += (a * sqrt(2) * cos(t)) / (pow(sin(t), 2) + 1);
        waypoint.position.y += (a * sqrt(2) * cos(t) * sin(t)) / (pow(sin(t), 2) + 1);

        waypoints.push_back(waypoint);
    }

    return waypoints;
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    std::cout << "Select a trajectory:\n";
    std::cout << "1. Straight Line\n";
    std::cout << "2. Helical\n";
    std::cout << "3. Cube Orientation\n";
    std::cout << "4. Eight figure\n";

    int choice;
    std::cin >> choice;

    geometry_msgs::Pose start_pose = move_group_interface.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;

    switch (choice) {
        case 1:
            waypoints = generateStraightLineTrajectory(start_pose);
            break;
        case 2:
            waypoints = generateHelicalTrajectory(start_pose);
            break;
        case 3:
            waypoints = generateCubeOrientationTrajectory(start_pose);
            break;
        case 4:
            waypoints = generateFigureEightTrajectory(start_pose);
            break;
     
        default:
            std::cout << "Invalid choice!" << std::endl;
            return 0;
    }

    // Execute the chosen trajectory
    executeTrajectory(waypoints, move_group_interface);

    return 0;
}
