#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h> // Add this include for PLY file support
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/organized_fast_mesh.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);
}

bool save_pointcloud(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    if (pcl_cloud->empty()) {
        res.success = false;
        res.message = "Point cloud is empty";
        return true;
    }

    std::string filename = "/home/cerlab/Desktop/aman/Files/output_pointcloud.ply";  // Replace with your predefined filename and location
    
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::OrganizedFastMesh<pcl::PointXYZ> organized_fast_mesh;
    organized_fast_mesh.setInputCloud(pcl_cloud);
    organized_fast_mesh.reconstruct(*mesh);

    pcl::io::savePLYFile(filename, *mesh);

    res.success = true;
    res.message = "Point cloud saved";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_saver_service");
    ros::NodeHandle nh;

    ros::Subscriber pc_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/output_cloud_topic", 1, pointCloudCallback);
    ros::ServiceServer service = nh.advertiseService("save_pointcloud_service", save_pointcloud);

    ros::spin();

    return 0;
}
