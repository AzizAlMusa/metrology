#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>

#include <vector>

#include <iostream>
#include <fstream>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/visualization/cloud_viewer.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;





int main (int argc, char** argv) {


//   ros::init(argc, argv, "publish_segments");
//   ros::NodeHandle nh;

//   ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("pcl_playground/segment_data", 1000);

  //std_msgs::String msg;
    



  //Loading .pcd file
  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<PointT> ("/home/abdulaziz/pcl_ws/src/pcl_playground/models/jig.pcd", *cloud) )
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  // parameter settings
  bool disable_transform = false;
  
  float voxel_resolution = 0.05f;
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);

  float seed_resolution = 20.0f;
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);

  float color_importance = 0.0f;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);

  float spatial_importance = 0.0f;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);

  float normal_importance = 5.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);
  

  // Setup supervoxel (super is a confusing name here)
  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  if (disable_transform) super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  

  // this map will be used to extract supervoxels, dont know how
  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);

  
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());


  return (0);
}


