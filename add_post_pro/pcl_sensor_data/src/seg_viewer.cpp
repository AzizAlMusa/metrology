


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>

#include <vector>
#include <math.h>

#include <iostream>
#include <fstream>
#include <cstdio>
#include <thread>
#include <sstream>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkPolyLine.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT> cloud;

    // // Open the CSV file
    // std::ifstream file("/home/abdulaziz/addpost_ws/segment2.csv");

    // // Read the data from the CSV file
    // std::string line;
    // while (std::getline(file, line))
    // {
    //     std::istringstream iss(line);
    //     float x, y, z;
    //     if (!(iss >> x >> y >> z)) { break; } // error

    //     // Add the point to the cloud
    //     PointT point;
    //     point.x = x;
    //     point.y = y;
    //     point.z = z;
    //     cloud.push_back(point);
    // }

    // // Close the file
    // file.close();

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer.addPointCloud(cloud.makeShared(), "cloud");

    // viewer->setBackgroundColor (1, 1, 1);
    // viewer->addCoordinateSystem (0.01, 0.0, 0.0, 0.0);
    // viewer->addPointCloud(cloud.makeShared(), "cloud");

    // // Set the background color
    // while (!viewer->wasStopped ()) {
 
    //             viewer->spinOnce (100);

    //         }
    return 0;
}
