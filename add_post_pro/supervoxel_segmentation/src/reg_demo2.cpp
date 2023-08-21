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
  // Generate mainCloud, a point cloud representing a cube
  pcl::PointCloud<pcl::PointXYZ>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZ>);
  float cubeSize = 1.0; // size of the cube
  int numPoints = 10000; // number of points to generate
  srand(time(NULL));
  for (int i = 0; i < numPoints; i++)
  {
    float x = cubeSize * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - cubeSize / 2.0;
    float y = cubeSize * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - cubeSize / 2.0;
    float z = cubeSize * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - cubeSize / 2.0;
    mainCloud->push_back(pcl::PointXYZ(x, y, z));
  }
  pcl::io::savePCDFileASCII("mainCloud.pcd", *mainCloud);

  // Extract partialCloud, a portion of the cube
  pcl::PointCloud<pcl::PointXYZ>::Ptr partialCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr indices(new pcl::PointIndices);
  for (int i = 0; i < numPoints; i++)
  {
    float x = cubeSize * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - cubeSize / 2.0;
    float y = cubeSize * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - cubeSize / 2.0;
    float z = cubeSize * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - cubeSize / 2.0;
    if (x > 0.0 && y > 0.0 && z > 0.0) // only keep points within the positive octant of the cube
    {
      partialCloud->push_back(pcl::PointXYZ(x, y, z));
      indices->indices.push_back(i);
    }
  }
  pcl::io::savePCDFileASCII("partialCloud.pcd", *partialCloud);

  // Create a PCL Visualizer object and add the mainCloud and partialCloud to it
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud(mainCloud, "mainCloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "mainCloud"); // red color
  viewer.addPointCloud(partialCloud, "partialCloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "partialCloud"); // green color

  // Keep the viewer open until it is manually closed
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }

  return 0;
}
