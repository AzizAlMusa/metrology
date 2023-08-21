/*
    This program is used to test the simulated laser scanner (scanCONTROL 2900-100 as of 7/15/2020) on a sample turntable.

    This program utilizes the services offered by the laser_processor node to combine a number of 2D laser profiles into one 
    pointcloud that should roughly reconstruct the object. 
*/


#include<ros/ros.h>
#include<ros/console.h>
#include<math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <pcl_sensor_data/StartRecording.h>
#include <pcl_sensor_data/StopRecording.h>
#include <pcl_sensor_data/ProcessPointcloud.h>
#include <pcl_sensor_data/RestartRecording.h>


#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
ros::NodeHandle *nh;
ros::ServiceClient *start_srv_client_ptr, *stop_srv_client_ptr, *restart_srv_client_ptr, *process_srv_client_ptr;
pcl_sensor_data::StartRecording *start_srv;
pcl_sensor_data::StopRecording *stop_srv;
pcl_sensor_data::ProcessPointcloud *process_srv;
pcl_sensor_data::RestartRecording *restart_srv;
static const std::string PLANNING_GROUP_ARM { "arm" };
static const std::string PLANNING_GROUP_TABLE { "turntable" };


// scanning pose arm = 
// x = 0.117398
// y = 0.018617
// z = 0.539448
// r = 2.027567
// p = 0.036714
// y = 1.588918

// scanning pose turntable = 
// x = 0.500001
// y = 0.000001
// z = 0.234951
// r = -0.000001
// p = 0.000004
// y = 0.590353

void move_to_start()
{

    tf::Quaternion q;

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_table(PLANNING_GROUP_TABLE);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ROS_INFO_NAMED("laser_scanner_movement", "table Planning frame: %s", move_group_table.getPlanningFrame().c_str());
	ROS_INFO_NAMED("laser_scanner_movement", "End effector link: %s", move_group_table.getEndEffectorLink().c_str());

    geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
    pose.position.x = 0.028;
    pose.position.y = -0.139;
    pose.position.z = 0.717;
    q.setEulerZYX( -1.453, -1.027, 3.019);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    std::cout << "Moving to scanning position" << std::endl;
    move_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    //Start laser here
    move_group.move();


    // geometry_msgs::Pose table_pose = move_group_table.getCurrentPose().pose;
    // tf::Quaternion q_t(table_pose.orientation.x, table_pose.orientation.y, table_pose.orientation.z, table_pose.orientation.w);
    // tf::Matrix3x3 m(q_t);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // q.setEuler(roll, pitch, 0.59);
    // // q.setEulerZYX(0.590353, 0.000004, -0.000001);
    // table_pose.orientation.x = q.x();
    // table_pose.orientation.y = q.y();
    // table_pose.orientation.z = q.z();
    // table_pose.orientation.w = q.w();

    // Let's try joint_space
    moveit::core::RobotStatePtr current_table = move_group_table.getCurrentState();
    std::vector<double> joint_group_positions;

    const moveit::core::JointModelGroup* joint_model_group =
    move_group_table.getCurrentState()->getJointModelGroup(PLANNING_GROUP_TABLE);
    current_table->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] +=3.14;
    move_group_table.setJointValueTarget(joint_group_positions);

    move_group_table.setMaxVelocityScalingFactor(0.4);
    move_group_table.setMaxAccelerationScalingFactor(0.3);
    moveit::planning_interface::MoveGroupInterface::Plan table_plan;
    move_group_table.plan(table_plan);


    start_srv->request.req = true;
    if(!start_srv_client_ptr->call(*start_srv))
    {
        ROS_ERROR("Couldn't start the recording");
        return;
    }
    ros::Duration(0.1).sleep();
    move_group_table.move();
    stop_srv->request.req = true;
    if(!stop_srv_client_ptr->call(*stop_srv))
    {
        ROS_ERROR("Couldn't stop the recording");
        return;
    }
    ros::Duration(0.2).sleep();
    process_srv->request.req = true;
    if(!process_srv_client_ptr->call(*process_srv))
    {
        ROS_ERROR("Couldn't process the recording");
        return;
    }
    return;

    // move_group_table.setPoseTarget(table_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan table_plan;
    // move_group_table.plan(table_plan);
    // move_group_table.move();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "turntable_scanning_demo");
    ROS_INFO("Starting node handle");
    nh = new ros::NodeHandle();

    ROS_INFO("Starting service clients");
    ros::ServiceClient start_srv_client, stop_srv_client, process_srv_client, restart_srv_client;

    start_srv_client = nh->serviceClient<pcl_sensor_data::StartRecording>("/laser_processor/start_recording");
    stop_srv_client = nh->serviceClient<pcl_sensor_data::StopRecording>("/laser_processor/stop_recording");
    process_srv_client = nh->serviceClient<pcl_sensor_data::ProcessPointcloud>("/laser_processor/process_pointcloud");
    restart_srv_client = nh->serviceClient<pcl_sensor_data::RestartRecording>("/laser_processor/restart_recording");

    start_srv_client_ptr = &start_srv_client;
    stop_srv_client_ptr = &stop_srv_client;
    process_srv_client_ptr = &process_srv_client;
    restart_srv_client_ptr = &restart_srv_client;

    ROS_INFO("Starting services");
    start_srv = new pcl_sensor_data::StartRecording();
    stop_srv = new pcl_sensor_data::StopRecording();
    process_srv = new pcl_sensor_data::ProcessPointcloud();
    restart_srv = new pcl_sensor_data::RestartRecording();

    ROS_INFO("Starting movement command");
    
    // return 0;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    move_to_start();

    return 0;
}