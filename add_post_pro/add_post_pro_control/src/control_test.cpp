#include <ros/ros.h>
#include <add_post_pro_control/GenerateViewPlan.h> // Include your custom service message
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "service_client_node");
    ros::NodeHandle nh;

    // Create a service client for the generate_view_plan service
    ros::ServiceClient client = nh.serviceClient<add_post_pro_control::GenerateViewPlan>("generate_viewpoint_trajectory");

    
    // Create a request message
    add_post_pro_control::GenerateViewPlan::Request req;
    add_post_pro_control::GenerateViewPlan::Response res;

    // Populate the request message with actual poses
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.5;
    pose1.position.y = 0.0;
    pose1.position.z = 0.5;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.0;
    pose1.orientation.w = 1.0;
    req.viewpoints.push_back(pose1);

    std::cout << "Pose will be sent!" << std::endl;

    
    // Send the request to the service
    if (client.call(req, res))
    {
        if (res.success)
        {
            ROS_INFO("Service call was successful");
        }
        else
        {
            ROS_ERROR("Service call failed");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}
