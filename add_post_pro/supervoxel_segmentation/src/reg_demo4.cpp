#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
  // Load the input point cloud from file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/abdulaziz/addpost_ws_new/src/add_post_pro/supervoxel_segmentation/models/reg_test.pcd", *cloud) == -1)
  {
    PCL_ERROR("Could not read input file\n");
    return -1;
  }

   // Load the input point cloud from file
  pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/abdulaziz/addpost_ws_new/src/add_post_pro/supervoxel_segmentation/models/jig.pcd", *refCloud) == -1)
  {
    PCL_ERROR("Could not read input file\n");
    return -1;
  }

  // Scale the point cloud by a factor of 2
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.scale(2.0f);
  pcl::transformPointCloud(*refCloud, *refCloud, transform);
  
  // Create a PCL Visualizer object and add the mainCloud and partialCloud to it
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud(cloud, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud"); // red color


  viewer.addPointCloud(refCloud, "refCloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "refCloud"); // red color

//   // Perform ICP registration between mainCloud and partialCloud
//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(cloud);
//   icp.setInputTarget(refCloud);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
//   icp.align(*aligned);

//   std::cout << "ICP fitness score: " << icp.getFitnessScore() << std::endl;

  
//   viewer.addPointCloud(aligned, "aligned");
//   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "aligned"); // green color


// // Compute surface normals and FPFH features for the source cloud
//   pcl::PointCloud<pcl::Normal>::Ptr normals_source(new pcl::PointCloud<pcl::Normal>);
//   pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_source(new pcl::PointCloud<pcl::FPFHSignature33>);
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree_source(new pcl::search::KdTree<pcl::PointXYZ>);
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_source;
//   normal_estimator_source.setInputCloud(cloud);
//   normal_estimator_source.setSearchMethod(search_tree_source);
//   normal_estimator_source.setKSearch(20);
//   normal_estimator_source.compute(*normals_source);
//   pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimator_source;
//   fpfh_estimator_source.setInputCloud(cloud);
//   fpfh_estimator_source.setInputNormals(normals_source);
//   fpfh_estimator_source.setSearchMethod(search_tree_source);
//   fpfh_estimator_source.setKSearch(20);
//   fpfh_estimator_source.compute(*features_source);


//   // Compute surface normals and FPFH features for the target cloud
//   pcl::PointCloud<pcl::Normal>::Ptr normals_target(new pcl::PointCloud<pcl::Normal>);
//   pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_target(new pcl::PointCloud<pcl::FPFHSignature33>);
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree_target(new pcl::search::KdTree<pcl::PointXYZ>);
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_target;
//   normal_estimator_target.setInputCloud(refCloud);
//   normal_estimator_target.setSearchMethod(search_tree_target);
//   normal_estimator_target.setKSearch(20);
//   normal_estimator_target.compute(*normals_target);
//   pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimator_target;
//   fpfh_estimator_target.setInputCloud(refCloud);
//   fpfh_estimator_target.setInputNormals(normals_target);
//   fpfh_estimator_target.setSearchMethod(search_tree_target);
//   fpfh_estimator_target.setKSearch(20);
//   fpfh_estimator_target.compute(*features_target);
  
//   std::cout << "FPFH Estimators Complete!" << std::endl;
//   // Initialize the Sample Consensus Initial Alignment (SAC-IA) registration algorithm
//   pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
  
//   std::cout << "Starting Consensus!" << std::endl;
//   // Set the input clouds and FPFH feature descriptors for the SAC-IA algorithm
//   sac_ia.setInputCloud(cloud);
//   sac_ia.setSourceFeatures(features_source);
//   sac_ia.setInputTarget(refCloud);
//   sac_ia.setTargetFeatures(features_target);

//   // Set the maximum correspondence distance for the SAC-IA algorithm (in meters)
//   sac_ia.setMaxCorrespondenceDistance(0.05);

//   // Set the minimum number of correspondences required for the SAC-IA algorithm
//   sac_ia.setMinSampleDistance(0.05);

//   // Set the number of iterations for the SAC-IA algorithm
//   sac_ia.setMaximumIterations(100);

//   // Align the two point clouds using the SAC-IA algorithm
//   pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source(new pcl::PointCloud<pcl::PointXYZ>);
//   sac_ia.align(*aligned_source);
//   std::cout << "Consensus Done!" << std::endl;
// //    Initialize the Iterative Closest Point (ICP) registration algorithm
// std::cout << "ICP Starting!" << std::endl;
//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

//   // Set the input clouds for the ICP algorithm
//   icp.setInputSource(aligned_source);
//   icp.setInputTarget(refCloud);

//   // Set the maximum correspondence distance for the ICP algorithm (in meters)
//   icp.setMaxCorrespondenceDistance(0.05);

//   // Set the maximum number of iterations for the ICP algorithm
//   icp.setMaximumIterations(100);

//   // Align the two point clouds using the ICP algorithm
//   pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_final(new pcl::PointCloud<pcl::PointXYZ>);
//   icp.align(*aligned_final);


//     std::cout << "ICP Done!" << std::endl;
//   viewer.addPointCloud(aligned_final, "aligned_final");
//   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "aligned_final"); // green color

// // Set up the ICP object
//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setMaxCorrespondenceDistance(0.005);
//   icp.setTransformationEpsilon(1e-8);
//   icp.setMaximumIterations(200);

//   // Set the input source and target point clouds for registration
//   icp.setInputSource(cloud);
//   icp.setInputTarget(refCloud);

//   // Align the source point cloud to the target point cloud
//   pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   icp.align(*aligned_cloud);
//   viewer.addPointCloud(aligned_cloud, "aligned_cloud");
//   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "aligned_cloud"); // green color

    while (!viewer.wasStopped())
  {
    viewer.spinOnce();
 } 

  // Save the output point cloud to file
//   pcl::io::savePCDFile<pcl::PointXYZ>("output_cloud.pcd", *cloud);

  return 0;
}
