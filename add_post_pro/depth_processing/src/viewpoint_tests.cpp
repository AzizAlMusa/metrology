#include <ros/ros.h>
#include <add_post_pro_control/GenerateViewPlan.h> // Include your custom service message
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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
    ros::init(argc, argv, "service_client_node");
    ros::NodeHandle nh;

    // Create a service client for the generate_view_plan service
    ros::ServiceClient client = nh.serviceClient<add_post_pro_control::GenerateViewPlan>("generate_viewpoint_trajectory");

    std::cout << "HELLO first" << std::endl;
    // Create a request message
    add_post_pro_control::GenerateViewPlan::Request req;
    add_post_pro_control::GenerateViewPlan::Response res;
    // Populate the request message with actual poses
    // geometry_msgs::Pose pose1;
    // pose1.position.x = 0.5;
    // pose1.position.y = 0.0;
    // pose1.position.z = 0.5;
    // pose1.orientation.x = 0.0;
    // pose1.orientation.y = 0.0;
    // pose1.orientation.z = 0.0;
    // pose1.orientation.w = 1.0;
    // req.viewpoints.push_back(pose1);

    // geometry_msgs::Pose pose2;
    // pose2.position.x = 0.0;
    // pose2.position.y = 0.5;
    // pose2.position.z = 0.5;
    // pose2.orientation.x = 0.0;
    // pose2.orientation.y = 0.0;
    // pose2.orientation.z = 0.0;
    // pose2.orientation.w = 1.0;
    // req.viewpoints.push_back(pose2);

        // Define the circle parameters
    double circle_radius = 0.5;
    int num_waypoints = 8;

    // Calculate and create the waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    for (int i = 0; i < num_waypoints; ++i)
    {
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_waypoints);
        geometry_msgs::PoseStamped waypoint = calculateCircleWaypoint(circle_radius, angle);
        req.viewpoints.push_back(waypoint.pose);
    }


    std::cout << "HELLO second" << std::endl;
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
