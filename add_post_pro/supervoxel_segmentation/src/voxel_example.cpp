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




 void visualize (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr & viewer);



void printVector(std::vector<std::vector <double> > data, std::string filename){

   ofstream MyFile(filename);
      //Simply for priting ^this thing
   for (int i = 0; i < data.size(); i++)
    {
        for (int j = 0; j < data[i].size(); j++)
        {   
              if ( true ){
              std::cout << data[i][j] << ",";
              MyFile << data[i][j] << ",";
            }
        }

        std::cout << std::endl;
        MyFile << std::endl;
    }
     MyFile.close();
}


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
  

  // setup visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  viewer->addCoordinateSystem (0.01, 0.0, 0.0, 0.0);
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  
  for (size_t i = 0; i < labeled_voxel_cloud->size(); i++)
  {
      pcl::PointXYZL point = labeled_voxel_cloud->points[i];
      std::cout << "Point " << i << ": " << point.x << " " << point.y << " " << point.z << " " << point.label << std::endl;
    }
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //We have this disabled so graph is easy to see, ` to see supervoxel normals
  //viewer->addPointCloudNormals<PointNT> (sv_normal_cloud,1,2.0f, "supervoxel_normals");


  //extract supervoxel adjacency list (in the form of a multimap of label adjacencies)
  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);


  //iterate through the multimap, creating a point cloud of the centroids of each supervoxel’s neighbors
   //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  
  //This is the payload for the publisher
  std::vector<std::vector <double> > waypoints;
  std::vector<std::vector <double> > viewpoints;
  for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); )
  {
    //First get the label
    std::uint32_t supervoxel_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    
    // ROS_INFO_STREAM(supervoxel->centroid_);
    // ROS_INFO_STREAM(supervoxel->normal_);

     
    // here the the supervoxel centroids and normals are taken and push into the waypoints variable
    std::vector<double> current_centroid {supervoxel->centroid_.x, supervoxel->centroid_.y, supervoxel->centroid_.z, 
                    supervoxel->normal_.normal_x, supervoxel->normal_.normal_y, supervoxel->normal_.normal_z};
    
    double h = 0.25;
    double viewpoint_x = supervoxel->centroid_.x - supervoxel->normal_.normal_x * h;
    double viewpoint_y = supervoxel->centroid_.y - supervoxel->normal_.normal_y * h;
    double viewpoint_z = supervoxel->centroid_.z - supervoxel->normal_.normal_z * h;

    std::vector<double> viewpoint {viewpoint_x, viewpoint_y, viewpoint_z, 
                    supervoxel->normal_.normal_x, supervoxel->normal_.normal_y, supervoxel->normal_.normal_z};

    //ROS_INFO_STREAM(current_centroid);
    waypoints.push_back(current_centroid);
    viewpoints.push_back(viewpoint);

    for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }


    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }

  printVector(waypoints, "waypoints.txt");
  printVector(viewpoints, "viewpoints.txt");


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }


  return (0);
}



void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer::Ptr & viewer)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    for (auto adjacent_itr = adjacent_supervoxel_centers.begin (); adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
    {
        points->InsertNextPoint (supervoxel_center.data);
        points->InsertNextPoint (adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    // Add the points to the dataset
    polyData->SetPoints (points);
    polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
    for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
        polyLine->GetPointIds ()->SetId (i,i);
    cells->InsertNextCell (polyLine);
    // Add the lines to the dataset
    //polyData->SetLines (cells);
    viewer->addModelFromPolyData (polyData,supervoxel_name);
}