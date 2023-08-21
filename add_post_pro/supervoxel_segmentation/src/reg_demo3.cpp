#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <ctime>
#include <cmath>

int main()
{
  // Generate mainCloud, a point cloud representing a sphere
  pcl::PointCloud<pcl::PointXYZ>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZ>);
  float radius = 1.0; // radius of the sphere
  int numPoints = 10000; // number of points to generate
  srand(time(NULL));
  for (int i = 0; i < numPoints; i++)
  {
    float theta = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 * M_PI;
    float phi = acos(2.0 * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0);
    float x = radius * sin(phi) * cos(theta);
    float y = radius * sin(phi) * sin(theta);
    float z = radius * cos(phi);
    mainCloud->push_back(pcl::PointXYZ(x, y, z));
  }
  pcl::io::savePCDFileASCII("mainCloud.pcd", *mainCloud);

  // Extract partialCloud, a portion of the sphere
  pcl::PointCloud<pcl::PointXYZ>::Ptr partialCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr indices(new pcl::PointIndices);
  for (int i = 0; i < numPoints; i++)
  {
    float theta = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 * M_PI;
    float phi = acos(2.0 * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0);
    if (phi < M_PI / 4.0) // only keep points within the lower quarter of the sphere
    {
      float x = radius * sin(phi) * cos(theta);
      float y = radius * sin(phi) * sin(theta);
      float z = radius * cos(phi);
      partialCloud->push_back(pcl::PointXYZ(x, y, z));
      indices->indices.push_back(i);
    }
  }
  pcl::io::savePCDFileASCII("partialCloud.pcd", *partialCloud);

  // Extract partialCloud, a portion of the sphere
  pcl::PointCloud<pcl::PointXYZ>::Ptr scannedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr indices2(new pcl::PointIndices);
  for (int i = 0; i < numPoints; i++)
  {
    float theta = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 * M_PI;
    float phi = acos(2.0 * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0);
    if (phi < M_PI / 4.0) // only keep points within the lower quarter of the sphere
    {
      float x = radius * sin(phi) * cos(theta) + 1.0;
      float y = radius * sin(phi) * sin(theta) + 1.0;
      float z = radius * cos(phi);
      scannedCloud->push_back(pcl::PointXYZ(x, y, z));
      indices2->indices.push_back(i);
    }
  }


  // Define a random rotation matrix
  Eigen::Matrix3f rotationMatrix;
  float angle = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * M_PI; // random angle in radians
  Eigen::Vector3f axis((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5, 
                      (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5,
                      (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5); // random axis
  axis.normalize();
  rotationMatrix = Eigen::AngleAxisf(angle, axis);

  // Define a random translation vector
  Eigen::Vector3f translation;
  translation(0) = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5;
  translation(1) = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5;
  translation(2) = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5;

  // Combine the rotation and translation into an affine transformation
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translate(translation);
  transform.rotate(rotationMatrix);

  // Apply the rotation and translation to the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*scannedCloud, *transformedCloud, transform);

  pcl::io::savePCDFileASCII("partialCloud.pcd", *scannedCloud);

  // Create a PCL Visualizer object and add the mainCloud and partialCloud to it
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud(mainCloud, "mainCloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "mainCloud"); // red color
  viewer.addPointCloud(partialCloud, "partialCloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "partialCloud"); // green color

  viewer.addPointCloud(scannedCloud, "scannedCloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "scannedCloud"); // green color

//   // Perform ICP registration between mainCloud and partialCloud
//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(scannedCloud);
//   icp.setInputTarget(mainCloud);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
//   icp.align(*aligned);

//   std::cout << "ICP fitness score: " << icp.getFitnessScore() << std::endl;

//   // Save aligned point cloud to file
//   pcl::io::savePCDFileASCII("alignedCloud.pcd", *aligned);
  
//   viewer.addPointCloud(aligned, "aligned");
//   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "aligned"); // green color
  // Keep the viewer open until it is manually closed
  
  // Define the colors for the two point clouds
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> redColor(scannedCloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> greenColor(mainCloud, 0, 255, 0);
  
    // Register the two point clouds and update the viewer at each iteration
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

    // Create an ICP object and set its parameters
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(50);

    viewer.addPointCloud(aligned, "aligned");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "aligned"); // green color
    for (int i = 0; i < 50; ++i)
    {
        icp.setInputSource(scannedCloud);
        icp.setInputTarget(mainCloud);
        icp.align(*aligned);

        // Convert the final transformation matrix to an affine transformation
        Eigen::Affine3f finalTransform;
        finalTransform.matrix() = icp.getFinalTransformation();
        // Update the viewer with the aligned point cloud and the current transformation matrix
        viewer.updatePointCloud(aligned, greenColor, "cloud2");
        viewer.updatePointCloudPose("cloud2", finalTransform);

        // viewer.spinOnce(10);
        // Sleep for a short time to slow down the animation
        // boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    // Wait for the viewer to close
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(10);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
  return 0;
}
