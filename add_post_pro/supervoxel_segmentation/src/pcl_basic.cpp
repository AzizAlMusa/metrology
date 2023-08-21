#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

 void foo (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
   //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
   //... populate cloud

  //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->addPointCloud(cloud, "stuff");
   
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
   {
    
   }
    ROS_INFO_STREAM("TEST");


 }



int main (int argc, char** argv) {




  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<PointT> ("/home/abdulaziz/pcl_ws/src/pcl_playground/models/bunny.pcd", *cloud) )
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }
  
  ROS_INFO_STREAM("Entering visualizer");
  foo(cloud);

  
//   pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
//   std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}