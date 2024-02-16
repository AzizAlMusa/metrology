#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>


std::string output_mesh_filename = "/home/cerlab/Desktop/output_mesh.ply";

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
    ROS_INFO("Starting Mesh reconstruction ... ");
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Create a mesh object
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    // pcl::GreedyProjectionTriangulation<pcl::PointXYZ> gp3;

    // gp3.setSearchRadius(0.025);
    // gp3.setMu(2.5);
    // gp3.setMaximumNearestNeighbors(100);
    // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    // gp3.setMinimumAngle(M_PI/18); // 10 degrees
    // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    // gp3.setNormalConsistency(true);

    // // Initialize OrganizedFastMesh object
    pcl::OrganizedFastMesh<pcl::PointXYZ> organized_fast_mesh;
    organized_fast_mesh.setInputCloud(pcl_cloud);
    organized_fast_mesh.reconstruct(*mesh);
    // gp3.setInputCloud(pcl_cloud);
    // gp3.reconstruct(*mesh);

    // Save the mesh locally
    if (pcl::io::savePLYFile(output_mesh_filename, *mesh) == 0) {
        ROS_INFO("Mesh saved to %s", output_mesh_filename.c_str());
    } 
    else {
        ROS_ERROR("Failed to save mesh!");
}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_generator");
    ros::NodeHandle nh;

    ros::Subscriber pc_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/depth_processor_realsense_cpp/combined_cloud", 1, pointCloudCallback);

    ros::spin();

    return 0;
}