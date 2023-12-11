#include <ros/ros.h>
#include <add_post_pro_control/GenerateViewPlan.h> // Include your custom service message
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <random>

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
    poseMsg.header.frame_id = "turntable_plate"; // Replace with your desired frame
    poseMsg.pose.position = position;
    poseMsg.pose.orientation = tf2::toMsg(quaternion);

    return poseMsg;
}

geometry_msgs::PoseStamped calculatePose(const geometry_msgs::Point& position)
{
    // Calculate the direction vector (z-axis) pointing towards the center (0, 0, 0)
    tf2::Vector3 zAxis(-position.x, -position.y, -position.z);
    zAxis.normalize();

    // Define a downward pointing vector (towards the x-y plane)
    tf2::Vector3 downVector(0, 0, -1);

    // Calculate the y-axis as the cross product of the z-axis and the down vector
    tf2::Vector3 yAxis = zAxis.cross(downVector);
    yAxis.normalize();

    // Calculate the x-axis as the cross product of the y-axis and the z-axis
    tf2::Vector3 xAxis = yAxis.cross(zAxis);

    // Create the rotation matrix from the x, y, and z axes
    tf2::Matrix3x3 rotationMatrix(xAxis.x(), yAxis.x(), zAxis.x(),
                                  xAxis.y(), yAxis.y(), zAxis.y(),
                                  xAxis.z(), yAxis.z(), zAxis.z());

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
    lookAt.z = 0.0; // Replace with your desired z-coordinate

    geometry_msgs::PoseStamped target_pose = calculatePose(position, lookAt);


    return target_pose;
}

// Function to calculate a waypoint on a sphere
geometry_msgs::PoseStamped calculateSphereWaypoint(double radius, double theta, double phi)
{
    // Convert spherical coordinates to Cartesian coordinates
    geometry_msgs::Point position;
    position.x = radius * sin(theta) * cos(phi);
    position.y = radius * sin(theta) * sin(phi);
    position.z = radius * cos(theta);

    // Define the look-at point (center of the sphere)
    geometry_msgs::Point lookAt;
    lookAt.x = 0.0;
    lookAt.y = 0.0;
    lookAt.z = 0.0;

    // Calculate and return the pose
    return calculatePose(position, lookAt);
}

// Function to generate n random poses on a sphere of radius r
// with theta between π/3 and π/6
std::vector<geometry_msgs::PoseStamped> generateSpherePoses(int n, double radius)
{
    std::vector<geometry_msgs::PoseStamped> poses;

    // Set up random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> theta_dist(M_PI / 12, M_PI / 3);  // Theta between π/6 and π/3
    std::uniform_real_distribution<> phi_dist(0, 2 * M_PI);  // Phi between 0 and 2π

    for (int i = 0; i < n; ++i)
    {
        // Generate random values for theta and phi
        double theta = theta_dist(gen);
        double phi = phi_dist(gen);

        geometry_msgs::PoseStamped waypoint = calculateSphereWaypoint(radius, theta, phi);
        poses.push_back(waypoint);
    }
    return poses;
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
    // pose1.position.x = 0.0;
    // pose1.position.y = 0.0;
    // pose1.position.z = 0.7;
    // pose1.orientation.x = 0.0;
    // pose1.orientation.y = 0.0;
    // pose1.orientation.z = 0.0;
    // pose1.orientation.w = 1.0;
    // req.viewpoints.push_back(pose1);

    // geometry_msgs::Pose pose2;
    // pose2.position.x = 0.0;
    // pose2.position.y = 0.0;
    // pose2.position.z = 0.7;
    // pose2.orientation.x = 0.0;
    // pose2.orientation.y = 0.0;
    // pose2.orientation.z = 0.0;
    // pose2.orientation.w = 1.0;
    // req.viewpoints.push_back(pose2);

    // Define the circle parameters
    double circle_radius = 0.5;
    int num_waypoints = 8;

    // // Calculate and create the waypoints
    // std::vector<geometry_msgs::Pose> waypoints;
    // for (int i = 0; i < num_waypoints; ++i)
    // {
    //     double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_waypoints);
    //     geometry_msgs::PoseStamped waypoint = calculateCircleWaypoint(circle_radius, angle);
    //     req.viewpoints.push_back(waypoint.pose);
    // }

    // Generate sphere poses
    int num_sphere_waypoints = 10; // Number of waypoints
    double sphere_radius = 0.5; // Sphere radius
    std::vector<geometry_msgs::PoseStamped> sphere_waypoints = generateSpherePoses(num_sphere_waypoints, sphere_radius);

    // Add the sphere waypoints to the request
    for (auto& waypoint : sphere_waypoints)
    {
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










// --------------------------- OLD ---------------------
// #include <ros/ros.h>
// #include <add_post_pro_control/GenerateViewPlan.h> // Include your custom service message
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// geometry_msgs::PoseStamped calculatePose(const geometry_msgs::Point& position, const geometry_msgs::Point& lookAt)
// {
//     // Calculate the direction vector from position to look-at point
//     tf2::Vector3 directionVector(lookAt.x - position.x, lookAt.y - position.y, lookAt.z - position.z);
//     directionVector.normalize();
//     // Calculate the rotation matrix manually
//     tf2::Vector3 upVector(0, 0, 1); // Define an up vector
//     tf2::Vector3 rightVector = directionVector.cross(upVector); // Calculate the right vector
//     rightVector.normalize();
//     upVector = directionVector.cross(rightVector); // Recalculate the up vector
//     tf2::Matrix3x3 rotationMatrix(
//         rightVector.getX(), upVector.getX(), directionVector.getX(),
//         rightVector.getY(), upVector.getY(), directionVector.getY(),
//         rightVector.getZ(), upVector.getZ(), directionVector.getZ()
//     );
//     // Convert the rotation matrix to a quaternion
//     tf2::Quaternion quaternion;
//     rotationMatrix.getRotation(quaternion);
//     // Create a PoseStamped message to represent the pose
//     geometry_msgs::PoseStamped poseMsg;
//     poseMsg.header.stamp = ros::Time::now();
//     poseMsg.header.frame_id = "base_link"; // Replace with your desired frame
//     poseMsg.pose.position = position;
//     poseMsg.pose.orientation = tf2::toMsg(quaternion);
//     return poseMsg;
// }
// // Function to calculate a waypoint on a circle
// geometry_msgs::PoseStamped calculateCircleWaypoint(double radius, double angle)
// {
//     // Define the position and look-at point
//     geometry_msgs::Point position;
//     position.x = radius * cos(angle);
//     position.y = radius * sin(angle);
//     position.z = 0.5; // Constant Z-coordinate
//     geometry_msgs::Point lookAt;
//     lookAt.x = 0.0; // Replace with your desired x-coordinate
//     lookAt.y = 0.0; // Replace with your desired y-coordinate
//     lookAt.z = 0.0
//     ; // Replace with your desired z-coordinate
//     geometry_msgs::PoseStamped target_pose = calculatePose(position, lookAt);
//     return target_pose;
// }
// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "service_client_node");
//     ros::NodeHandle nh;
//     // Create a service client for the generate_view_plan service
//     ros::ServiceClient client = nh.serviceClient<add_post_pro_control::GenerateViewPlan>("generate_viewpoint_trajectory");
//     std::cout << "HELLO first" << std::endl;
//     // Create a request message
//     add_post_pro_control::GenerateViewPlan::Request req;
//     add_post_pro_control::GenerateViewPlan::Response res;
//     // Populate the request message with actual poses
//     // geometry_msgs::Pose pose1;
//     // pose1.position.x = 0.0;
//     // pose1.position.y = 0.0;
//     // pose1.position.z = 0.7;
//     // pose1.orientation.x = 0.0;
//     // pose1.orientation.y = 0.0;
//     // pose1.orientation.z = 0.0;
//     // pose1.orientation.w = 1.0;
//     // req.viewpoints.push_back(pose1);
//     // geometry_msgs::Pose pose2;
//     // pose2.position.x = 0.0;
//     // pose2.position.y = 0.0;
//     // pose2.position.z = 0.7;
//     // pose2.orientation.x = 0.0;
//     // pose2.orientation.y = 0.0;
//     // pose2.orientation.z = 0.0;
//     // pose2.orientation.w = 1.0;
//     // req.viewpoints.push_back(pose2);
//     // Define the circle parameters
//     double circle_radius = 0.5;
//     int num_waypoints = 8;
//     // Calculate and create the waypoints
//     std::vector<geometry_msgs::Pose> waypoints;
//     for (int i = 0; i < num_waypoints; ++i)
//     {
//         double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_waypoints);
//         geometry_msgs::PoseStamped waypoint = calculateCircleWaypoint(circle_radius, angle);
//         req.viewpoints.push_back(waypoint.pose);
//     }
//     std::cout << "HELLO second" << std::endl;
//     // Send the request to the service
//     if (client.call(req, res))
//     {
//         if (res.success)
//         {
//             ROS_INFO("Service call was successful");
//         }
//         else
//         {
//             ROS_ERROR("Service call failed");
//         }
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service");
//         return 1;
//     }
//     return 0;
// }