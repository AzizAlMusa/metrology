#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <add_post_pro_control/GenerateViewPlan.h>
#include <eigen_conversions/eigen_msg.h>

namespace rvt = rviz_visual_tools;

class ViewpointController
{
private:
    ros::NodeHandle node_handle_;
    ros::Publisher trajectory_publisher_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    ros::ServiceServer trajectory_service_;
  
public:

    ViewpointController() : move_group_("arm")
    {
        // Initialize the ROS node
        node_handle_ = ros::NodeHandle();
        

    
        // Initialize the publisher
        trajectory_publisher_ = node_handle_.advertise<moveit_msgs::RobotTrajectory>("robot_scan_trajectory", 1);

        // Advertise the service
        trajectory_service_ = node_handle_.advertiseService("generate_viewpoint_trajectory", &ViewpointController::executeTrajectory, this);


        std::cout << "Service initialized, waiting to receive request for trajectory computation!" << std::endl;


     
    }


    moveit_msgs::RobotTrajectory computeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints)
    {
        moveit_msgs::RobotTrajectory trajectory;
        
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED("Cartesian Fraction Success", "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

        moveit_visual_tools::MoveItVisualTools visual_tools("world");

        // Check if a valid robot state is available (without the async spinner on 4 thread this fails)
        if (!move_group_.getCurrentState()) {
            std::cout << "Unable to fetch current robot state for visualization. Skipping this step." << std::endl;
        } else {
            // Check if JointModelGroup is available
            const moveit::core::JointModelGroup* joint_model_group = move_group_.getCurrentState()->getJointModelGroup("arm");
            
            if (joint_model_group == nullptr) {
                std::cout << "Unable to fetch JointModelGroup for visualization. Skipping this step." << std::endl;
            } else {
       
                
                // Perform the visualization
                visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel(joint_model_group->getLinkModelNames().back()), joint_model_group, rvt::LIME_GREEN);
                // visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("true_laser"), joint_model_group, rvt::LIME_GREEN);
                for (std::size_t i = 0; i < waypoints.size(); ++i) {
                    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
                }
                
            }
        }


        visual_tools.trigger();
        visual_tools.prompt("Press next to execute.");

        // move_group_.execute(trajectory);
        return trajectory;
    }

    bool executeTrajectory(add_post_pro_control::GenerateViewPlan::Request& req,
                        add_post_pro_control::GenerateViewPlan::Response& res)
    {   
        std::cout << "Trajectory request received, computing now..." << std::endl;
        std::vector<geometry_msgs::Pose> viewpoints = req.viewpoints;

        // Iterate over each viewpoint
        for (size_t i = 0; i < viewpoints.size(); ++i) {
            // Retrieve the robot's current state
            moveit::core::RobotState start_state(*move_group_.getCurrentState());
            const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup("arm");

            // Get the current pose of the robot
            Eigen::Isometry3d eigen_pose = start_state.getGlobalLinkTransform(joint_model_group->getLinkModel(joint_model_group->getLinkModelNames().back()));

            // Convert Eigen pose to ROS Pose
            geometry_msgs::Pose current_pose;
            tf::poseEigenToMsg(eigen_pose, current_pose);

            // Create a segment from the current position to the current viewpoint
            std::vector<geometry_msgs::Pose> segment;
            segment.push_back(current_pose); // Current robot position
            segment.push_back(viewpoints[i]); // Target viewpoint

            // Set the start state for the move group
            move_group_.setStartState(start_state);

            // Compute and execute the trajectory for the current segment
            moveit_msgs::RobotTrajectory trajectory = computeTrajectory(segment);
            trajectory_publisher_.publish(trajectory);

            // Wait for user input or a signal to continue
            std::cout << "Press Enter to continue to the next segment..." << std::endl;
            std::cin.ignore();
        }

        res.success = true;
        return true;
    }




};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_robot_control_service");

    // Start the spinner with 4 threads
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Services will listen through this class
    ViewpointController viewpoint_controller;

    // Spin the node to keep it alive and responsive to service requests
    ros::waitForShutdown();

    return 0;
}


