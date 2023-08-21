
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <realsense_cam/FetchOneDepth.h>
#include <realsense_cam/FetchOneRGB.h>
#include <realsense_cam/FindWorkspaceCenter.h>
#include <realsense_cam/FindBoxPoints.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <fstream>

#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

std::string planningName = "base_link";
double alignment_value;
ros::ServiceClient rgb_client;
ros::ServiceClient workspace_center_client;
ros::ServiceClient box_points_client;
realsense_cam::FetchOneRGB rgb_srv;
realsense_cam::FetchOneRGB::Request rgb_req;
realsense_cam::FetchOneRGB::Response rgb_res;
ros::Subscriber alignment_sub;

void prompt()
{
    printf("Press Enter \n");
    getchar();
}

// box_points: 1D Array of points inside workspace
void kd_tree_planning(sensor_msgs::PointCloud2 points, std::vector<geometry_msgs::PointStamped> boundary_points)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(points, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *new_cloud);

}
void find_box()
{
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("EEF_tool_mount");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    realsense_cam::FindWorkspaceCenter center_srv;
    realsense_cam::FindBoxPoints box_srv;
    geometry_msgs::PoseStamped pose, curr_pose, start_pose;
    geometry_msgs::PointStamped point, temp_point;
    sensor_msgs::PointCloud out_pointcloud;
    static const std::string PLANNING_GROUP { "arm" };
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    tf::Quaternion q;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::PointStamped initial_point, transformed_point;
    tf::StampedTransform transform_stamped;
    tf::TransformListener listener;
	tf::Transformer transformer;

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    move_group.setMaxVelocityScalingFactor(0.1);
    workspace_center_client.call(center_srv);
    prompt();
    pose.pose.position = center_srv.response.point.point;
    pose.pose.position.z += 0.55;
    // pose.pose.position.y -= 0.1;
    pose.header.frame_id = "base_link";
    q.setEulerZYX(-M_PI*1.00,0, -M_PI);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    move_group.setEndEffectorLink("D415_color_optical_frame");
    move_group.setPoseTarget(pose);
    std::cout << "First calculated point: " << pose << std::endl; 
	move_group.move();
    ros::Duration(1.0).sleep();
    // workspace_center_client.call(center_srv);
    box_points_client.call(box_srv);
    kd_tree_planning(box_srv.response.points, box_srv.response.boundary_points);
    prompt();
    // return;
    // workspace_center_client.call(center_srv);

    // pose.pose.position = center_srv.response.point.point;
    // pose.pose.position.y -= 0.1;
    // pose.pose.position.z += 0.55;
    // pose.header.frame_id = "base_link";
    // std::cout << "Tweaking Z-angle by " << center_srv.response.z_change << " radians." << std::endl;
    // std::cout << "Tweaking X-angle by " << center_srv.response.x_change << " radians to " << -M_PI*1.03+center_srv.response.x_change << std::endl;
    // q.setEulerZYX(-M_PI *1.05 - center_srv.response.z_change,0, -M_PI);
    // pose.pose.orientation.x = q.x();
    // pose.pose.orientation.y = q.y();
    // pose.pose.orientation.z = q.z();
    // pose.pose.orientation.w = q.w();
    // move_group.setEndEffectorLink("D415_color_optical_frame");
    // move_group.setPoseTarget(pose);
	// move_group.move();
    // workspace_center_client.call(center_srv);
    // // return; 
    // prompt();
    // 2nd change

    // pose.pose.position = center_srv.response.point.point;
    // pose.pose.position.z += 0.55;
    // pose.header.frame_id = "base_link";
    // std::cout << "Tweaking Z-angle by " << center_srv.response.z_change << " radians." << std::endl;
    // q.setEulerZYX(-M_PI - center_srv.response.z_change,0, -M_PI);
    // pose.pose.orientation.x = q.x();
    // pose.pose.orientation.y = q.y();
    // pose.pose.orientation.z = q.z();
    // pose.pose.orientation.w = q.w();
    // move_group.setEndEffectorLink("D415_color_optical_frame");
    // move_group.setPoseTarget(pose);
	// move_group.move();

    // ros::Duration(1.0).sleep();
    // prompt();
    // 3rd change
    // workspace_center_client.call(center_srv);
    // pose.pose.position = center_srv.response.point.point;
    // pose.pose.position.z += 0.55;
    // pose.header.frame_id = "base_link";
    // // std::cout << "Tweaking Z-angle by " << center_srv.response.z_change << " radians." << std::endl;
    // // q.setEulerZYX(-M_PI - center_srv.response.z_change,0, -M_PI);
    // pose.pose.orientation.x = q.x();
    // pose.pose.orientation.y = q.y();
    // pose.pose.orientation.z = q.z();
    // pose.pose.orientation.w = q.w();
    // move_group.setEndEffectorLink("D415_color_optical_frame");
    // move_group.setPoseTarget(pose);
	// move_group.move();
    move_group.setEndEffectorLink("EEF_sensor_holder");
    start_pose = move_group.getCurrentPose();
    // while alignment is less than some value
    // while(true){
    //     prompt();
    //     waypoints.clear();
    //     box_points_client.call(box_srv);

    //     kd_tree_planning(box_srv.response.points, box_srv.response.boundary_points);
    //     move_group.clearPathConstraints();
    //     move_group.setEndEffectorLink("EEF_sensor_holder");
    //     q.setEulerZYX(-M_PI/2,0, -M_PI);
    //     pose.pose.orientation.x = q.x();
    //     pose.pose.orientation.y = q.y();
    //     pose.pose.orientation.z = q.z();
    //     pose.pose.orientation.w = q.w();


    //     move_group.clearPathConstraints();
    //     move_group.clearPoseTargets();

    //     // curr_pose.pose.position.z-=.01;
    //     // waypoints.push_back(curr_pose.pose);
    //     // move_group.setStartStateToCurrentState();
    //     std::cout << "box points.size: " << box_srv.response.points.size() << std::endl;
        
    //     pcl::PCLPointCloud2 pcl_pc2;
    //     // pcl_conversions::toPCL(box_srv_response.)
    //     for (int i=0;i<box_srv.response.points.size();i=i+5)
    //     {
    //         if (i==0 || i==5)
    //         {
    //             continue;
    //         }
    //         // for (int j=-5; j<=5;j++)
    //         // {
                
    //         // }
    //         pose.pose.position = box_srv.response.points[i].point;
    //         pose.pose.position.z +=.180;
    //         // pose.pose.position.x +=.00;
    //         // move_group.setPoseTarget(pose);
    //         // std::cout << pose.pose << std::endl;
    //         // move_group.move();
    //         // break;
    //         waypoints.push_back(pose.pose);
    //         // std::cout << pose << std::endl;
    //     }
    //     // waypoints.push_back(start_pose.pose);
    //     move_group.setMaxVelocityScalingFactor(0.1);
    //     moveit_msgs::RobotTrajectory trajectory;
    //     const double jump_threshold = 0;
    //     const double eef_step = 0.008;
    //     double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //     visual_tools.deleteAllMarkers();
    //     visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    //     for (std::size_t i = 0; i < waypoints.size(); ++i)
    //         visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    //     visual_tools.trigger();
    //     // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    //     my_plan.trajectory_ = trajectory;
    //     prompt();
    //     std::ofstream traj_file;
    //     traj_file.open("traj.txt");

    //     traj_file << my_plan.trajectory_ << std::endl;
    //     traj_file.close();
    //     move_group.execute(my_plan);
    //     prompt();
    //     move_group.setPoseTarget(start_pose.pose);
	//     move_group.move();
    // }
    return;
}

void move_to_start(ros::NodeHandle nh)
{
    geometry_msgs::Pose pose, curr_pose, next_pose;
    static const std::string PLANNING_GROUP { "arm" };
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    tf::Quaternion q;
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setNamedTarget("depowdering_tracking");
    move_group.move();
    prompt();
    pose = move_group.getCurrentPose().pose;
    // pose.orientation.y +=.1;
    std::cout << pose << std::endl;
    q.setEulerZYX(M_PI, 0, M_PI);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    move_group.setEndEffectorLink("D415_color_optical_frame");
    pose = move_group.getCurrentPose().pose;
    std::cout << "eef_pose:\n" << pose << std::endl;
    pose.position.y -=.084;
    pose.position.z +=.065;
    // std::cout << "pose2:\n" << pose << std::endl; 
    move_group.setPoseTarget(pose);
    move_group.move();

    ros::Duration(0.5).sleep();
    curr_pose = pose;

    find_box();
}

void alignment_callback(const std_msgs::String::ConstPtr& alignment_message )
{
    ROS_INFO("alignment value is ", alignment_message->data.c_str());
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "rough_depowdering_box_coverage");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    alignment_sub=nh.subscribe("point_cloud_match/alignment",20 ,alignment_callback);
    rgb_client = nh.serviceClient<realsense_cam::FetchOneRGB>("/realsense_services/next_rgb");
    workspace_center_client = nh.serviceClient<realsense_cam::FindWorkspaceCenter>("/realsense_services/find_workspace_center");
    box_points_client = nh.serviceClient<realsense_cam::FindBoxPoints>("/realsense_services/find_box_points");

    // Testing kd_tree
    realsense_cam::FindBoxPoints box_srv;
    box_points_client.call(box_srv);
    kd_tree_planning(box_srv.response.points, box_srv.response.boundary_points);
    return 0;
    // End testing
    move_to_start(nh);
}



/* -------------------------------------- OLD -------------------------------------- */

// #include <ros/ros.h>
// #include <ros/console.h>
// #include <math.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit_msgs/Constraints.h>
// #include <moveit_msgs/OrientationConstraint.h>

// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/point_cloud_conversion.h>

// #include <realsense_cam/FetchOneDepth.h>
// #include <realsense_cam/FetchOneRGB.h>
// #include <realsense_cam/FindWorkspaceCenter.h>
// #include <realsense_cam/FindBoxPoints.h>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>

// #include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>
// #include <tf/transform_datatypes.h>

// std::string planningName = "base_link";
// ros::ServiceClient rgb_client;
// ros::ServiceClient workspace_center_client;
// ros::ServiceClient box_points_client;
// realsense_cam::FetchOneRGB rgb_srv;
// realsense_cam::FetchOneRGB::Request rgb_req;
// realsense_cam::FetchOneRGB::Response rgb_res;

// void find_box()
// {
//     realsense_cam::FindWorkspaceCenter center_srv;
//     realsense_cam::FindBoxPoints box_srv;
//     geometry_msgs::PoseStamped pose, curr_pose;
//     geometry_msgs::PointStamped point;
//     sensor_msgs::PointCloud out_pointcloud;
//     static const std::string PLANNING_GROUP { "arm" };
//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     tf::Quaternion q;
//     std::vector<geometry_msgs::Pose> waypoints;
//     geometry_msgs::PointStamped initial_point, transformed_point;
//     tf::StampedTransform transform_stamped;
//     tf::TransformListener listener;
// 	tf::Transformer transformer;
//     // We can print the name of the reference frame for this robot.
//     ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

//     // We can also print the name of the end-effector link for this group.
//     ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

//     workspace_center_client.call(center_srv);

//     pose.pose.position = center_srv.response.point.point;
//     pose.pose.position.z += 0.45;
//     pose.header.frame_id = "base_link";
//     q.setEulerZYX(-M_PI,0, -M_PI);
//     pose.pose.orientation.x = q.x();
//     pose.pose.orientation.y = q.y();
//     pose.pose.orientation.z = q.z();
//     pose.pose.orientation.w = q.w();
//     move_group.setEndEffectorLink("color");
//     move_group.setPoseTarget(pose);
//     std::cout << "First calculated point: " << pose << std::endl; 
// 	move_group.move();
//     ros::Duration(1.0).sleep();

//     workspace_center_client.call(center_srv);

//     pose.pose.position = center_srv.response.point.point;
//     pose.pose.position.z += 0.45;
//     pose.header.frame_id = "base_link";
//     q.setEulerZYX(-M_PI,0, -M_PI);
//     pose.pose.orientation.x = q.x();
//     pose.pose.orientation.y = q.y();
//     pose.pose.orientation.z = q.z();
//     pose.pose.orientation.w = q.w();
//     move_group.setEndEffectorLink("color");
//     move_group.setPoseTarget(pose);
// 	move_group.move();


//     box_points_client.call(box_srv);
//     move_group.clearPathConstraints();
//     move_group.setEndEffectorLink("EEF_tool_mount");
//     q.setEulerZYX(-M_PI,0, -M_PI);
//     pose.pose.orientation.x = q.x();
//     pose.pose.orientation.y = q.y();
//     pose.pose.orientation.z = q.z();
//     pose.pose.orientation.w = q.w();


//     move_group.clearPathConstraints();
//     move_group.clearPoseTargets();
//     curr_pose = move_group.getCurrentPose();
//     // curr_pose.pose.position.z-=.01;
//     // waypoints.push_back(curr_pose.pose);
//     move_group.setStartStateToCurrentState();
//     for (int i=0;i<box_srv.response.points.size();i=i+2)
//     {
//         pose.pose.position = box_srv.response.points[i].point;
//         pose.pose.position.z +=.15;

//         waypoints.push_back(pose.pose);
//         // std::cout << pose << std::endl;
//     }

//     move_group.setMaxVelocityScalingFactor(0.1);
//     moveit_msgs::RobotTrajectory trajectory;
//     const double jump_threshold = 0;
//     const double eef_step = 0.001;
//     double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//     my_plan.trajectory_ = trajectory;
// 	move_group.execute(my_plan);
//     return;
// }

// void move_to_start(ros::NodeHandle nh)
// {
//     geometry_msgs::Pose pose, curr_pose, next_pose;
//     static const std::string PLANNING_GROUP { "arm" };
//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//     move_group.setNamedTarget("rough_depowdering_b120");
//     move_group.move();

//     pose = move_group.getCurrentPose().pose;
//     pose.position.y-=.06;
//     pose.position.z+=.1;
//     move_group.setPoseTarget(pose);
//     move_group.move();

//     curr_pose = pose;
//     find_box();
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "rough_depowdering_box_coverage");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(2);
//     spinner.start();
//     rgb_client = nh.serviceClient<realsense_cam::FetchOneRGB>("/realsense_services/next_rgb");
//     workspace_center_client = nh.serviceClient<realsense_cam::FindWorkspaceCenter>("/realsense_services/find_workspace_center");
//     box_points_client = nh.serviceClient<realsense_cam::FindBoxPoints>("/realsense_services/find_box_points");
//     move_to_start(nh);
// }