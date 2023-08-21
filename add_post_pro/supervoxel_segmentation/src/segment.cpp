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


#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/moment_of_inertia_estimation.h>


//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

#include <math.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;





#include<pcl_playground/segment.h>



      
Segment::Segment(pcl::Supervoxel<PointT>::Ptr supervoxel, std::uint32_t supervoxel_label)
:supervoxel_(supervoxel), cloud(new PointLCloudT){       
            



    auto something = supervoxel->voxels_;



    cloud->resize(supervoxel->voxels_->size());

     
    for (size_t i = 0; i < supervoxel->voxels_->points.size(); i++) {
        
        cloud->points[i].x = supervoxel->voxels_->points[i].x;
        cloud->points[i].y = supervoxel->voxels_->points[i].y;
        cloud->points[i].z = supervoxel->voxels_->points[i].z;
        cloud->points[i].label = supervoxel_label;
    }      
    
    supervoxel->getCentroidPoint(centroid);
    supervoxel->getCentroidPointNormal(normal);

    Segment::project_cloud();
    Segment::calculate_features();
}


PointLCloudT::Ptr Segment::get_cloud(){
    return cloud;
}

pcl::Supervoxel<PointT>::Ptr Segment::get_supervoxel(){
    return supervoxel_;
}

PointT Segment::get_centroid(){
    return centroid;
}

PointNT Segment::get_normal(){
    return normal;
}


Eigen::Vector3f Segment::get_major_eig(){
    return major_vector;
};
Eigen::Vector3f Segment::get_middle_eig(){
    return middle_vector;
};
Eigen::Vector3f Segment::get_minor_eig(){


    float x = centroid.x - mass_center(0);
    float y = centroid.y - mass_center(1);
    float z = centroid.z - mass_center(2);

    Eigen::Vector3f lookAt(x, y, z);


    float dot_product =  lookAt.dot(minor_vector);
    
    if (dot_product < 0){
        return -1 * minor_vector;
    }
    return minor_vector;
};

Eigen::Vector3f Segment::get_mass_center(){
    return mass_center;
};


void Segment::calculate_features(){

    feature_extractor.setInputCloud (projected_cloud.makeShared());
    feature_extractor.compute ();
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    //Consider use for OBB

}

void Segment::project_cloud(){
    // Plane : P.n - d = 0
  double planeDistance = 0.15;
  // n = normal vector; d = lowest distance
  float nx = -normal.normal_x; // -ve sign denoting outward normal direction
  float ny = -normal.normal_y;
  float nz = -normal.normal_z;


  float segment_distance = centroid.x*nx + centroid.y*ny + centroid.z*nz;
  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = nx;
  coefficients->values[1] = ny;
  coefficients->values[2] = nz;
  coefficients->values[3] = -segment_distance - planeDistance; //-sqrt(pow(centroid.x, 2) + pow(centroid.y, 2) + pow(centroid.z,2));//-lowestDist-planeDistance;

  PointLCloudT::Ptr cloud_projected (new PointLCloudT);

  // Create the filtering object
  pcl::ProjectInliers<PointLT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  projected_cloud = *cloud_projected;
}


PointLCloudT Segment::get_projected_cloud(){

    return projected_cloud;
}


std::vector<float> Segment::get_raw_pose(){

    std::vector<float> raw_pose;

    
    //Position
    raw_pose.push_back(mass_center(0));
    raw_pose.push_back(mass_center(1));
    raw_pose.push_back(mass_center(2));

    //X axis (scan direction)
    raw_pose.push_back(major_vector(0));
    raw_pose.push_back(major_vector(1));
    raw_pose.push_back(major_vector(2));

    //Y axis laser line width
    raw_pose.push_back(middle_vector(0));
    raw_pose.push_back(middle_vector(1));
    raw_pose.push_back(middle_vector(2));


    //z axis (view direction/normal)
    auto normal_direc = get_minor_eig();
    raw_pose.push_back(normal_direc(0));
    raw_pose.push_back(normal_direc(1));
    raw_pose.push_back(normal_direc(2));

    return raw_pose;
}
