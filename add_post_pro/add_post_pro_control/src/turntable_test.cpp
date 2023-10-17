#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rvt = rviz_visual_tools;

geometry_msgs::PoseStamped calculatePose(const geometry_msgs::Point& position, const geometry_msgs::Point& lookAt)
{
    // Calculate the direction vector from position to look-at point
    tf2::Vector3 directionVector(lookAt.x - position.x, lookAt.y - position.y, lookAt.z - position.z);
    directionVector.normalize();

    // Calculate the rotation matrix manually
    tf2::Vector3 upVector(0, 0, 1); // Define an up vector
    tf2::Vector3 rightVector = directionVector.cross(upVector); // Calculate the right vector
    rightVector.normalize();
    upVector = directionVector.cross(rightVector); // Recalculate the up vector

    tf2::Matrix3x3 rotationMatrix(
        rightVector.getX(), upVector.getX(), directionVector.getX(),
        rightVector.getY(), upVector.getY(), directionVector.getY(),
        rightVector.getZ(), upVector.getZ(), directionVector.getZ()
    );

    // Convert the rotation matrix to a quaternion
    tf2::Quaternion quaternion;
    rotationMatrix.getRotation(quaternion);

    // Create a PoseStamped message to represent the pose
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = "base_link"; // Replace with your desired frame
    poseMsg.pose.position = position;
    poseMsg.pose.orientation = tf2::toMsg(quaternion);

    return poseMsg;
}

moveit_msgs::RobotTrajectory executeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints, moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    moveit_msgs::RobotTrajectory trajectory;
    
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);


    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm");
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("true_laser"), joint_model_group, rvt::LIME_GREEN);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press next to execute.");

    // move_group_interface.execute(trajectory);
    return trajectory;
}

// Function to calculate a waypoint on a circle
geometry_msgs::PoseStamped calculateCircleWaypoint(double radius, double angle)
{

    // Define the position and look-at point
    geometry_msgs::Point position;
    position.x = radius * cos(angle);
    position.y = radius * sin(angle);
    position.z = 0.5; // Constant Z-coordinate

    geometry_msgs::Point lookAt;
    lookAt.x = 0.0; // Replace with your desired x-coordinate
    lookAt.y = 0.0; // Replace with your desired y-coordinate
    lookAt.z = 0.32; // Replace with your desired z-coordinate

    geometry_msgs::PoseStamped target_pose = calculatePose(position, lookAt);


    return target_pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_circle_point_with_lookat");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Create a publisher for the robot_scan_trajectory topic
    ros::Publisher trajectory_publisher = node_handle.advertise<moveit_msgs::RobotTrajectory>("robot_scan_trajectory", 1);
    // Initialize MoveIt for your robot's arm
    moveit::planning_interface::MoveGroupInterface move_group("arm");

      // Define the circle parameters
    double circle_radius = 0.5;
    int num_waypoints = 8;

   // Calculate and create the waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    for (int i = 0; i < num_waypoints; ++i)
    {
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_waypoints);
        geometry_msgs::PoseStamped waypoint = calculateCircleWaypoint(circle_radius, angle);
        waypoints.push_back(waypoint.pose);
    }
    // Execute the trajectory with visualization
    moveit_msgs::RobotTrajectory trajectory = executeTrajectory(waypoints, move_group);

   

    // Publish the computed trajectory
    trajectory_publisher.publish(trajectory);

    ros::shutdown();
    return 0;
}
