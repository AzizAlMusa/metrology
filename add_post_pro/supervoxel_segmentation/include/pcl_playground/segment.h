#ifndef SEGMENT_H
#define SEGMENT_H

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


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

class Segment
{

private:

    //The labeled cloud of the segment
    PointLCloudT::Ptr cloud;

    //The labeled cloud of the segment
    PointLCloudT projected_cloud;


    //The supervoxel
    pcl::Supervoxel<PointT>::Ptr supervoxel_;

    //Centroid
    PointT centroid;

    //Normal
    PointNT normal;

  
    //Eigenvectors of a segment
    Eigen::Vector3f major_vector, middle_vector, minor_vector;

    //Mass Center
    Eigen::Vector3f mass_center;

    //Moment of intertia of pointcloud
    pcl::MomentOfInertiaEstimation <PointLT> feature_extractor;




public:
    
  
    //Constructor
    Segment(pcl::Supervoxel<PointT>::Ptr supervoxel, std::uint32_t supervoxel_label);


    //Mutators
    void calculate_features();

    void project_cloud();



    //Accessors
    PointLCloudT::Ptr get_cloud();

    pcl::Supervoxel<PointT>::Ptr get_supervoxel();

    PointT get_centroid();

    PointNT get_normal();


    Eigen::Vector3f get_major_eig();
    Eigen::Vector3f get_middle_eig();
    Eigen::Vector3f get_minor_eig();
    Eigen::Vector3f get_mass_center();

    PointLCloudT get_projected_cloud();

    std::vector<float> get_raw_pose();

};

#endif // SEGMENT_H