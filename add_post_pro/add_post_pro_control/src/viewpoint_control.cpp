#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <add_post_pro_control/GenerateViewPlan.h>

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
                visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("true_laser"), joint_model_group, rvt::LIME_GREEN);
                
                for (std::size_t i = 0; i < waypoints.size(); ++i) {
                    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
                }
                
            }
        }


        visual_tools.trigger();
        visual_tools.prompt("Press next to execute.");

        // move_group_interface.execute(trajectory);
        return trajectory;
    }

    bool executeTrajectory(add_post_pro_control::GenerateViewPlan::Request& req,
                          add_post_pro_control::GenerateViewPlan::Response& res)
    {   
        std::cout << "Trajectory request received, computing now..." << std::endl;
        std::vector<geometry_msgs::Pose> viewpoints = req.viewpoints;
        moveit_msgs::RobotTrajectory trajectory = computeTrajectory(viewpoints);
        trajectory_publisher_.publish(trajectory);
        
        // Set res.success to true explicitly
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


