#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>

#include <vector>
#include <math.h>

#include <iostream>
#include <fstream>
#include <cstdio>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/moment_of_inertia_estimation.h>



//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>


//custom segments class

#include <pcl_playground/segment.h>

//potato
// #include <pcl_playground/potato.h>

#include <chrono>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;


class VCC_Manager
{

    private:
        
        //The raw cloud from the input .pcd object
        PointCloudT::Ptr cloud;

        // Setup supervoxel (super is a confusing name here)
        pcl::SupervoxelClustering<PointT> superspace;

        // this map will be used to extract supervoxels, dont know how
        std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;


        //Viewer
        pcl::visualization::PCLVisualizer::Ptr viewer;

        //Labeled voxel Cloud
        PointLCloudT::Ptr labeled_voxel_cloud;

        //centroid cloud
        PointCloudT::Ptr voxel_centroid_cloud;

        //Normal cloud
        PointNCloudT::Ptr sv_normal_cloud;

        //Supervoxel adjanency
        std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;


        //Variables for initial waypoints used
        std::vector<std::vector <double> > waypoints;
        std::vector<std::vector <double> > viewpoints;

        //Moment of intertia feature extractor
        pcl::MomentOfInertiaEstimation <PointLT> feature_extractor;

        //Each segment as a pointcloud
        std::map<int, PointLCloudT::Ptr> map_of_segments;

        //Each Segment object in a label->Segment
        std::map<int, Segment> map_of_supervoxels;

        //Eigenvectors of a segment
        Eigen::Vector3f major_vector, middle_vector, minor_vector;

        //Segment Viewer Iterator
        int current_seg = 1;
        float seed_res = 0.3f;
        float voxel_res = 0.006;
        float normal_factor = 0.0;
        float spatial_factor = 0.0;
       
    public:

        // Constructor
        VCC_Manager(float voxel_resolution, float seed_resolution)
         :cloud (new PointCloudT), superspace(voxel_resolution, seed_resolution){
        
         }



        PointCloudT::Ptr get_cloud(){
            return cloud;
        }

        pcl::SupervoxelClustering<PointT> get_superspace(){
            return superspace;
        }
        
        void load_cloud(){
             //Loading .pcd file
            pcl::console::print_highlight ("Loading point cloud...\n");

            if (pcl::io::loadPCDFile<PointT> ("/home/abdulaziz/pcl_ws/src/pcl_playground/models/jig.pcd", *cloud) ){

                pcl::console::print_error ("Error loading cloud file!\n");
            
            } else {
                pcl::console::print_highlight ("Loaded point cloud successfully!\n");
            }
        }



        void create_superspace(int argc, char** argv){
             // parameter settings
            bool disable_transform = false;
            
          

            float color_importance = 0.0f;
            if (pcl::console::find_switch (argc, argv, "-c"))
                pcl::console::parse (argc, argv, "-c", color_importance);

            float spatial_importance = 1.0f;
            if (pcl::console::find_switch (argc, argv, "-z"))
                pcl::console::parse (argc, argv, "-z", spatial_importance);

            float normal_importance = 1.0f;
            if (pcl::console::find_switch (argc, argv, "-n"))
                pcl::console::parse (argc, argv, "-n", normal_importance);

          
            //setup the universe laws for the supervoxel clustering
            if (disable_transform) superspace.setUseSingleCameraTransform (false);

            superspace.setInputCloud (cloud);
            superspace.setColorImportance (color_importance);
            superspace.setSpatialImportance (spatial_importance);
            superspace.setNormalImportance (normal_importance);
            pcl::console::print_highlight ("superspace space initialized!\n");
            
        }




        void extract_superclusters(){

            pcl::console::print_highlight ("Extracting supervoxels!\n");
            superspace.extract (supervoxel_clusters);
            std::cout << cloud->size() << std::endl;

            pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
        
        }

        void create_fundamental_clouds(){
            
            //Cloud but labeled for each cluster
            labeled_voxel_cloud = superspace.getLabeledVoxelCloud ();

            //Centroid cloud
            voxel_centroid_cloud = superspace.getVoxelCentroidCloud();

            //Normals cloud
            sv_normal_cloud = superspace.makeSupervoxelNormalCloud (supervoxel_clusters);

        }


        int get_cluster_count(){
            return supervoxel_clusters.size();
        }


        void vectorize_centroids_normals(){

            for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); ) {
           
                //First get the label
                std::uint32_t supervoxel_label = label_itr->first;
                //Now get the supervoxel corresponding to the label
                pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);


                // here the the supervoxel centroids and normals are taken and push into the waypoints variable
                std::vector<double> current_centroid {supervoxel->centroid_.x, supervoxel->centroid_.y, supervoxel->centroid_.z, 
                                supervoxel->normal_.normal_x, supervoxel->normal_.normal_y, supervoxel->normal_.normal_z};
                
                double h = 0.25;
                double viewpoint_x = supervoxel->centroid_.x - supervoxel->normal_.normal_x * h;
                double viewpoint_y = supervoxel->centroid_.y - supervoxel->normal_.normal_y * h;
                double viewpoint_z = supervoxel->centroid_.z - supervoxel->normal_.normal_z * h;

                std::vector<double> viewpoint {viewpoint_x, viewpoint_y, viewpoint_z, 
                                supervoxel->normal_.normal_x, supervoxel->normal_.normal_y, supervoxel->normal_.normal_z};


                waypoints.push_back(current_centroid);
                viewpoints.push_back(viewpoint);

                //Now we make a name for this polygon
                std::stringstream ss;
                ss << "supervoxel_" << supervoxel_label;

                //Move iterator forward to next label
                label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);

                

            }
            
            
            
        }

        void save_waypoints(std::string filename){

            auto data = waypoints;
            ofstream MyFile(filename);
                //Simply for priting ^this thing
            for (int i = 0; i < data.size(); i++){
                
                for (int j = 0; j < data[i].size(); j++){   
                
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

        void save_viewpoints(std::string filename){
            
            auto data = viewpoints;
            ofstream MyFile(filename);
                //Simply for priting ^this thing
            for (int i = 0; i < data.size(); i++){
                
                for (int j = 0; j < data[i].size(); j++){   
                
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


        void save_viewpoints(std::vector<std::vector<float>> raw_poses, std::string filename){
            
            auto data = raw_poses;
            ofstream MyFile(filename);
                //Simply for priting ^this thing
            for (int i = 0; i < data.size(); i++){
                
                for (int j = 0; j < data[i].size(); j++){   
                
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

        void poses_to_file(){
            
            std::vector<std::vector<float> > pose_table;

            for (auto const&i : map_of_supervoxels){

                auto segment = i.second;

                auto pose = segment.get_raw_pose();

                pose_table.push_back(pose);

            }

            save_viewpoints(pose_table, "raw_poses.csv");

        }
        void store_supervoxels(){

            superspace.getSupervoxelAdjacency (supervoxel_adjacency);

            for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); ) {
           
                //First get the label
                std::uint32_t supervoxel_label = label_itr->first;
                //Now get the supervoxel corresponding to the label
                pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

              
                Segment segment(supervoxel, supervoxel_label );

                //Store it 
                map_of_supervoxels.insert({supervoxel_label, segment});

                //Now we make a name for this polygon
                std::stringstream ss;
                ss << "supervoxel_" << supervoxel_label;

                //Move iterator forward to next label
                label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
              
            }
        }

        std::map<int, Segment> get_segment_map(){
            return map_of_supervoxels;
        }


        void populate_viewer(){
            int voxel_counter = 0;
            superspace.getSupervoxelAdjacency (supervoxel_adjacency);

            for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); ) {
           
                //First get the label
                std::uint32_t supervoxel_label = label_itr->first;
                //Now get the supervoxel corresponding to the label
                pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

                //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
                PointCloudT adjacent_supervoxel_centers;
        

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


        }


        void create_viewer(){
            // setup visualizer
      
            viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer->setBackgroundColor (1, 1, 1);
            viewer->addCoordinateSystem (0.01, 0.0, 0.0, 0.0);

           

            auto callback = std::bind(&VCC_Manager::KeyboardEventOccurred, this, std::placeholders::_1);
            viewer->registerKeyboardCallback(callback);

        }


        void view_clouds(){

           
            viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "voxel centroids");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.5, "voxel centroids");

            
            viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,6, "labeled voxels");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "labeled voxels");
        }





        void
            addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                            PointCloudT &adjacent_supervoxel_centers,
                                            std::string supervoxel_name,
                                            pcl::visualization::PCLVisualizer::Ptr & viewer){

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



        void run_viewer(){

            while (!viewer->wasStopped ()) {
 
                viewer->spinOnce (100);

            }
        }

        void segment_demo(){
            
            Segment seg = map_of_supervoxels.at(current_seg);
            view_segment(seg, current_seg);

            current_seg++;
        }

        void vcc_params_demo(){
            
           
          
            viewer->removeAllPointClouds();

            pcl::SupervoxelClustering<PointT> super2(voxel_res, seed_res);
            super2.setInputCloud (cloud);
            super2.setColorImportance (0.0f);
            super2.setSpatialImportance (spatial_factor);
            super2.setNormalImportance (normal_factor);


            superspace = super2;
            // seed_res /= 2;    

            extract_superclusters();
            create_fundamental_clouds();
            view_clouds();

        }

        void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event) {

            if (event.keyDown() && event.getKeySym() == "a"){

                std::cout << "Enter params: " << std::endl;
                std::string input;
                std::getline(std::cin, input);

                voxel_res = std::atof(input.substr(0, input.find(' ')).c_str());
                input = input.substr(input.find(' ') + 1);
                seed_res = std::atof(input.substr(0, input.find(' ')).c_str());
                input = input.substr(input.find(' ') + 1);
                spatial_factor = std::atof(input.substr(0, input.find(' ')).c_str());
                input = input.substr(input.find(' ') + 1);
                normal_factor = std::atof(input.c_str());

                vcc_params_demo();
            }


            if (event.keyDown()) {
            
            std::cout << "Key pressed: " << event.getKeySym() << std::endl;

            segment_demo();
         
            }

    
        }



        void view_segment(Segment segment, int i){
            
            std::string str = std::to_string(i);

            std::cout << str << std::endl;
            PointLCloudT::Ptr cloud = segment.get_cloud();


            
        
            //Cloud
            viewer->addPointCloud (cloud, "segment voxels"+ str);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,6, "segment voxels" + str);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "segment voxels"+ str);
            


            //Projection
        
            PointLCloudT proj_cloud = segment.get_projected_cloud();

            viewer->addPointCloud (proj_cloud.makeShared(), "segment projection" + str);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,6, "segment projection"+ str);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.75, "segment projection" + str);



            //Centroid Point
            PointLCloudT::Ptr centroid(new  PointLCloudT);
            std::uint32_t _label=2;
            PointLT center_point;

            center_point.x = segment.get_centroid().x;
            center_point.y = segment.get_centroid().y;
            center_point.z = segment.get_centroid().z;
            center_point.label = _label;

            centroid->push_back(center_point);

            viewer->addPointCloud (centroid, "centroid voxel" + str);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,12, "centroid voxel" + str);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "centroid voxel" + str);



            // EigneVectors at Centroid and Mass Center
            Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
            major_vector = segment.get_major_eig();
            middle_vector = segment.get_middle_eig();
            minor_vector = segment.get_minor_eig();
            mass_center = segment.get_mass_center();

            auto average_center = segment.get_centroid();
            
            float scale = sqrt( pow( average_center.x - mass_center(0) ,2) + pow(average_center.y - mass_center(1) ,2) + pow(average_center.z - mass_center(2) ,2));
            pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
            pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
            pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
            pcl::PointXYZ z_axis (minor_vector (0) * scale + mass_center (0), minor_vector (1)*scale + mass_center (1), minor_vector (2)*scale + mass_center (2));


            // x_axis.x *= 0.25;
            // x_axis.y *= 0.25;
            // x_axis.z *= 0.25;

            // y_axis.x *= 0.25;
            // y_axis.y *= 0.25;
            // y_axis.z *= 0.25;

            // z_axis.x *= 0.25;
            // z_axis.y *= 0.25;
            // z_axis.z *= 0.25;

            // viewer->addLine (center, x_axis , 1.0f, 0.0f, 0.0f, "major eigen vector" + str);
            // viewer->addLine (center, y_axis , 0.0f,  1.0f, 0.0f, "middle eigen vector" + str);
            viewer->addLine (center, z_axis , 0.0f, 0.0f,  1.0f, "minor eigen vector" + str);
        }


        void test(){
            std::cout << "Testing!" << std::endl;
        }
        

};


void check(){
    std::cout << "Working!" << std::endl;
}

 
int main(int argc, char** argv)
{


    float voxel_resolution = 0.003f;
    bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
    if (voxel_res_specified)
        pcl::console::parse (argc, argv, "-v", voxel_resolution);

    float seed_resolution = 0.06f;
    bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
    if (seed_res_specified)
        pcl::console::parse (argc, argv, "-s", seed_resolution);


    auto start = std::chrono::high_resolution_clock::now();
    // code to measure goes here
   
    VCC_Manager* Supercluster = new VCC_Manager(voxel_resolution, seed_resolution);

    Supercluster->load_cloud();
    Supercluster->create_superspace(argc, argv);
    Supercluster->extract_superclusters();

    Supercluster->create_fundamental_clouds();

    Supercluster->create_viewer();
    // Supercluster->view_clouds();



    //This is to extract a segment and view it
    //This is a pointcloud object of the supervoxel
    // auto seg = Supercluster->get_segment(1);
    // Supercluster->view_segment(seg);

    //What is this for???????
    // Supercluster->populate_viewer();

    //Picks these from supervoxel.centroids/normals
    Supercluster->vectorize_centroids_normals();

    Supercluster->store_supervoxels();



    auto  voxel_centroid_cloud = Supercluster->get_superspace().getVoxelCentroidCloud();
    

    int voxel_count = 0;

    
    std::cout << "TOTAL VOXELS = " << voxel_centroid_cloud->size() << std::endl;

    // Supercluster->save_viewpoints("viewpoints.csv");
    Supercluster->poses_to_file();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: " << duration.count() / 1000 << " ms" << std::endl;
    Supercluster->run_viewer();

  return 0;
}

