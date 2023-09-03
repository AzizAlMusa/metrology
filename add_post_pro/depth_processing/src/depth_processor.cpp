#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>

#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>


#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkLookupTable.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkSTLReader.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>


// This is for the GUI to change variable values
#include <dynamic_reconfigure/server.h>
#include <depth_processing/FiltersConfig.h>

// threads and time
#include <thread>
#include <chrono>

#include <unistd.h>


class DepthProcessor
{
public:
     
      DepthProcessor() : nh_("~")
    {   

        std::string package_path = ros::package::getPath("depth_processing");
        stl_file_path = package_path + "/saved_models/resampled_cube.stl";
        // Subscribers
        cloud_sub_ = nh_.subscribe("/robot/depth_camera/points", 1, &DepthProcessor::pointCloudCallback, this);
        depth_image_sub_ = nh_.subscribe("/robot/depth_camera/image_raw", 1, &DepthProcessor::depthImageCallback, this);

        // Publishers
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
        adjusted_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("adjusted_cloud", 1);
        retrieved_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("retrieved_cloud", 1);
        combined_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
        aligned_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
        reference_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reference_cloud", 1);
        intersection_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("intersection_cloud", 1);

        // Services
        load_service_ = nh_.advertiseService("load_mesh", &DepthProcessor::loadReferenceMesh, this);
        capture_service_ = nh_.advertiseService("capture_measurement", &DepthProcessor::captureMeasurement, this);
        retrieve_service_ = nh_.advertiseService("get_measurement", &DepthProcessor::getMeasurement, this);
        align_service_ = nh_.advertiseService("align_measurements", &DepthProcessor::alignPointClouds, this);
        publish_all_service_ = nh_.advertiseService("publish_all_measurements", &DepthProcessor::publishAllMeasurements, this);
        intersect_service_ = nh_.advertiseService("intersect_point_clouds", &DepthProcessor::intersectPointClouds, this);
        mesh_service_ = nh_.advertiseService("generate_mesh", &DepthProcessor::generateMesh, this);
        deviation_service_ = nh_.advertiseService("deviation_mesh", &DepthProcessor::visualizeDeviationHeatmap, this);

        // Load launch file parameters
        nh_.param("use_voxel_filter", use_voxel_filter_, true);
        nh_.param("use_sor_filter", use_sor_filter_, true);
        nh_.param("use_passthrough_filter", use_passthrough_filter_, true);
        nh_.param("use_sac_registration", use_sac_registration_, true);
        
        // Transforms
        tf_listener_ = new tf::TransformListener();

        // Dynamic reconfigure - GUI for changing variables
        f_ = boost::bind(&DepthProcessor::dynamicReconfigCallback, this, _1, _2);
        server_.setCallback(f_);

        // Reference model
        surface_cloud = createSurfaceBoxPointCloud(x_size, y_size, z_size, resolution);

       
        // Initialize PCL viewer
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);  // The parameter specifies the size of the axes

        // Run the viewer on a separate thread
        // std::thread viewerThread(&DepthProcessor::runViewer, this);
        // viewerThread.detach();


  
    }  

    bool loadReferenceMesh(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
       // Initialize nominal_cloud if it hasn't been initialized yet
        if (!nominal_cloud) {
            nominal_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
            ROS_INFO("Point cloud object initialized.");
        }

        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();

        // Make sure the file path is correct
        if (stl_file_path.empty()) {
            res.success = false;
            res.message = "STL file path is empty.";
            return false;
        }

        reader->SetFileName(stl_file_path.c_str());
        ROS_INFO("Reading STL file from path: %s", stl_file_path.c_str());

        vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
        reader->Update();

        // Fill in cloud data
        for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); i++)
        {
            double p[3];
            polydata->GetPoint(i, p);

            pcl::PointXYZ point;
            point.x = p[0];
            point.y = p[1];
            point.z = p[2] + 0.25;

            nominal_cloud->points.push_back(point);
        }

        nominal_cloud->width = nominal_cloud->points.size();
        nominal_cloud->height = 1;
        nominal_cloud->is_dense = true;

        // Populate the nominal_mesh cloud field
        pcl::toPCLPointCloud2(*nominal_cloud, nominal_mesh.cloud);

        // Populate the nominal_mesh polygons field
        for (vtkIdType i = 0; i < polydata->GetNumberOfCells(); i++)
        {
            vtkCell* cell = polydata->GetCell(i);
            pcl::Vertices vertices;

            for (vtkIdType j = 0; j < cell->GetNumberOfPoints(); j++)
            {
                vertices.vertices.push_back(cell->GetPointId(j));
            }

            nominal_mesh.polygons.push_back(vertices);
        }

        if (nominal_mesh.polygons.size() == 0)
        {

            res.success = false;
            res.message = "Failed to load STL file.";
            return false;
        }
        else
        {
      
            // viewer->addPointCloud<pcl::PointXYZ>(nominal_cloud, "nominal_cloud");
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "nominal_cloud");  // Purple: R=0.5, G=0, B=0.5

    
            viewer->spinOnce(10000);
            res.success = true;
            res.message = "Successfully loaded STL file.";
            return true;
        }

      
    }




    /**
     * @brief Service callback to capture the current point cloud measurement.
     * 
     * This function captures the current processed point cloud (if available) 
     * and stores it in the measurements_ vector for later retrieval.
     * 
     * @param req Service request (empty for this service).
     * @param res Service response. Contains a success flag and a message.
     * @return true if the service completed successfully, false otherwise.
     */
bool captureMeasurement(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
     if (adjusted_cloud_) // Check if there's any data
    {   
        tf::StampedTransform transform;
        try
        {
            tf_listener_->lookupTransform("world", "camera_link", ros::Time(0), transform);
            measurements_.push_back(adjusted_cloud_);  // Store the transformed point cloud
            transforms_.push_back(transform);  // Store the transform

            res.success = true;
            res.message = "Measurement captured successfully!";
            return true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("TF exception:\n%s", ex.what());
            res.success = false;
            res.message = "Failed to capture measurement due to TF exception.";
            return false;
        }
    }
    else
    {
        res.success = false;
        res.message = "No adjusted point cloud data available!";
        return false;
    }
}


    /**
     * @brief Service callback to retrieve and publish the last captured point cloud measurement.
     * 
     * This function retrieves the last captured point cloud from the measurements_ vector 
     * and publishes it on the retrieved_cloud topic for visualization or further processing.
     * 
     * @param req Service request (empty for this service).
     * @param res Service response. Contains a success flag and a message.
     * @return true if the service completed successfully, false otherwise.
     */
    bool getMeasurement(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        // Check if there are any measurements stored
        if (!measurements_.empty())
        {
            // Convert the last stored point cloud to a ROS message
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*measurements_.back(), output);
            // pcl::toROSMsg(*measurements_[0], output);
            
            // Publish the point cloud on the retrieved_cloud topic
            retrieved_cloud_pub_.publish(output);
            
            // Set the success flag and message for the service response
            res.success = true;
            res.message = "Published the last captured measurement on the retrieved_cloud topic.";
            return true;
        }
        else
        {
            // If no measurements are available, set the service response accordingly
            res.success = false;
            res.message = "No measurements available.";
            return false;
        }
    }

bool publishAllMeasurements(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

     // Check if there are any measurements stored
    if (!measurements_.empty())
    {
        // Ensure the two vectors are of the same size
        if (measurements_.size() != transforms_.size())
        {
            ROS_ERROR("Mismatch in sizes between measurements and transforms. Cannot proceed.");
            res.success = false;
            res.message = "Internal error: Mismatch in sizes.";
            return false;
        }

        // Create an empty combined point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Iterate through all measurements and append them to the combined_cloud
        for (size_t i = 0; i < measurements_.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_ros::transformPointCloud(*measurements_[i], *transformed_cloud, transforms_[i]);
            *combined_cloud += *transformed_cloud;
        }

        // Convert the combined point cloud to a ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*combined_cloud, output);
        output.header.frame_id = "world";  // We'll use the world frame since it's a combined cloud

        // Publish the combined point cloud on the combined_cloud_pub_ topic
        combined_cloud_pub_.publish(output);
        
        // Set the success flag and message for the service response
        res.success = true;
        res.message = "Published the combined measurements on the combined_cloud topic.";
        return true;
    }
    else
    {
        // If no measurements are available, set the service response accordingly
        res.success = false;
        res.message = "No measurements available.";
        return false;
    }

}


    void depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    ROS_INFO("Received depth image with width: %d and height: %d", image_msg->width, image_msg->height);
    // TODO: Any direct processing on the depth image if required
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // ROS_INFO("Received a point cloud message.");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    


    // Voxel Grid Filter
    if (use_voxel_filter_)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_filter_leaf_size_, voxel_filter_leaf_size_, voxel_filter_leaf_size_);
        voxel_filter.filter(*cloud_filtered);
    }

    // Statistical Outlier Removal
    if (use_sor_filter_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
        sor_filter.setInputCloud(cloud_filtered);
        sor_filter.setMeanK(sor_mean_k_);
        sor_filter.setStddevMulThresh(sor_std_dev_);
        sor_filter.filter(*cloud_filtered);
    }

    // Passthrough Filter
    if (use_passthrough_filter_)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(passthrough_limit_min_, passthrough_limit_max_);
        pass.filter(*cloud_filtered);
    }


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = cloud_msg->header;  // Use the same header as the input cloud for consistency
    current_cloud_ = cloud_filtered;
    cloud_pub_.publish(output);

    
    // Adjust the point cloud based on the transform between camera_link and camera_link_adjusted

    tf::TransformListener listener;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("camera_link", "camera_link_adjusted", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("camera_link", "camera_link_adjusted", ros::Time(0), transform);
        pcl_ros::transformPointCloud(*cloud_filtered, *cloud_adjusted, transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("Transform error: %s", ex.what());
        return;
    }

    sensor_msgs::PointCloud2 adjusted_output;
    pcl::toROSMsg(*cloud_adjusted, adjusted_output);
    adjusted_output.header = cloud_msg->header;  // Use the same header as the input cloud for consistency
    adjusted_cloud_ = cloud_adjusted;
    adjusted_cloud_pub_.publish(adjusted_output);  // If you have a different publisher for adjusted cloud, use that.
    
}


bool alignPointClouds(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Convert the PCL point cloud to ROS message
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(*nominal_cloud, ros_msg);
    
    // Set the frame_id to "world"
    ros_msg.header.frame_id = "world";

    reference_cloud_pub_.publish(ros_msg);

    if (measurements_.empty())
    {
        res.success = false;
        res.message = "No measurements available.";
        return false;
    }

    if (measurements_.size() != transforms_.size())
    {
        ROS_ERROR("Mismatch in sizes between measurements and transforms. Cannot proceed.");
        res.success = false;
        res.message = "Internal error: Mismatch in sizes.";
        return false;
    }

    // Register keyboard callback for your viewer
    viewer->registerKeyboardCallback(&DepthProcessor::keyboardEventOccurred, *this);


    pcl::PointCloud<pcl::PointXYZ>::Ptr final_registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Initialization of ICP and its parameters
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setMaximumIterations(500);

    // This extremely important for good quality registrations. The nature of the problem means measurements are not that far off.
    // a 2 cm correspondence should be generous and avoids misaligned registrations.
    icp.setMaxCorrespondenceDistance(0.001); 

    icp.setTransformationEpsilon(1e-9);

    // viewer->addPointCloud<pcl::PointXYZ>(surface_cloud, "surface_cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 1.0, "surface_cloud");  // Light Blue: R=0.5, G=0.5, B=1.0
    // register each cloud in measurements_
    for (size_t i = 0; i < measurements_.size(); ++i)
    {   
        // Transform clouds from sensor frame to world frame through transforms_
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl_ros::transformPointCloud(*measurements_[i], *transformed_cloud, transforms_[i]);


        pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
 

        Eigen::Matrix4f initial_transform_eigen;
        pcl_ros::transformAsMatrix(transforms_[i], initial_transform_eigen);

        // 
        icp.setInputSource(transformed_cloud);
        icp.setInputTarget(nominal_cloud);
        icp.align(*registered_cloud);

        if (icp.hasConverged())
        {
            // Add the aligned point cloud to the final cloud
            *final_registered_cloud += *registered_cloud;

            // Update the viewer
            // viewer->removeAllPointClouds();
            // viewer->removePointCloud("registered_cloud");
            // viewer->removePointCloud("transformed_cloud");

            // viewer->addPointCloud<pcl::PointXYZ>(registered_cloud, "registered_cloud");
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0.5, "registered_cloud");  // Light Blue: R=0.5, G=0.5, B=1.0

            
            // viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "transformed_cloud");
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "transformed_cloud");  // Purple: R=0.5, G=0, B=0.5
   

            // viewer->spinOnce(5000);

            // viewer->removePointCloud("registered_cloud");
            // viewer->removePointCloud("transformed_cloud");
  

        }
        else

        {
            ROS_WARN("ICP failed to converge for measurement %zu.", i);
        }


    }



    // viewer->addPointCloud<pcl::PointXYZ>(final_registered_cloud, "final_registered_cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "final_registered_cloud");  // Purple: R=0.5, G=0, B=0.5
   

    // viewer->spinOnce(10000);
    final_registered_cloud_ = final_registered_cloud;
    // Publishing the combined registered cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*final_registered_cloud, output);
    output.header.frame_id = "world";
    aligned_cloud_pub_.publish(output);

    res.success = true;
    res.message = "Aligned and published the combined measurements.";
    return true;


}



bool intersectPointClouds(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(nominal_cloud);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 0.01;  // Set appropriate radius

    for (const auto& point : final_registered_cloud_->points)
    {
        if (kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
        intersection_cloud->points.push_back(point);
        }
    }

    intersection_cloud->width = intersection_cloud->points.size();
    intersection_cloud->height = 1;
    intersection_cloud->is_dense = true;

    intersection_cloud_ = intersection_cloud;
    sensor_msgs::PointCloud2 intersection_msg;
    pcl::toROSMsg(*intersection_cloud, intersection_msg);
    intersection_msg.header.frame_id = "world";
    intersection_pub_.publish(intersection_msg);

    res.success = true;
    res.message = "Intersection completed and published.";
    return true;
}



bool generateMesh(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (nominal_cloud->empty())
    {
        ROS_WARN("Final registered cloud is empty. Cannot generate mesh.");
        return false;
    }

    // Normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(normal_estimation_k_search_);
    norm_est.setInputCloud(nominal_cloud);
    norm_est.compute(*normals);

    // Concatenate XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*nominal_cloud, *normals, *cloud_with_normals);

    // Initialize Poisson reconstruction object
    pcl::Poisson<pcl::PointNormal> poisson;

    // Set Poisson parameters
   // Set Poisson parameters
    poisson.setDepth(poisson_depth_);
    poisson.setScale(poisson_scale_);
    poisson.setSolverDivide(poisson_solver_divide_);
    poisson.setIsoDivide(poisson_iso_divide_);
    poisson.setConfidence(poisson_confidence_);
    poisson.setPointWeight(poisson_point_weight_);
    poisson.setSamplesPerNode(poisson_samples_per_node_);
    poisson.setManifold(poisson_manifold_);

    poisson.setInputCloud(cloud_with_normals);

    // Perform reconstruction
    poisson.reconstruct(scanned_mesh);

    if (scanned_mesh.polygons.size() > 0)
    {
        res.success = true;
        res.message = "Mesh successfully generated.";
        
        // Visualize the mesh
        viewer->addPolygonMesh(scanned_mesh, "mesh");
        viewer->spinOnce(2000);
        viewer->removePolygonMesh("mesh");
    }
    else
    {
        res.success = false;
        res.message = "Failed to generate mesh.";
    }
    return true;
}

vtkSmartPointer<vtkPolyData> convertToVtkPolyData(const pcl::PolygonMesh& pcl_mesh) {
    // Convert pcl::PointCloud to vtkPoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_mesh.cloud, *cloud);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& point : cloud->points) {
        points->InsertNextPoint(point.x, point.y, point.z);
    }

    // Convert polygons to vtkCellArray
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    for (const auto& polygon : pcl_mesh.polygons) {
        polygons->InsertNextCell(polygon.vertices.size());
        for (const auto& vertex : polygon.vertices) {
            polygons->InsertCellPoint(vertex);
        }
    }

    // Create a vtkPolyData object and set its points and polygons
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    polydata->SetPolys(polygons);

    return polydata;
}


bool visualizeDeviationHeatmap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    
    // viewer->addPointCloud(intersection_cloud_, "intersection_cloud");
    // viewer->addPointCloud(surface_cloud, "surface_cloud");

    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "intersection_cloud"); 
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "surface_cloud"); 

    // Create a KD-tree for nominal_cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(nominal_cloud);
    
    vtkSmartPointer<vtkFloatArray> deviations = vtkSmartPointer<vtkFloatArray>::New();
    deviations->SetName("Deviations");

    // Calculate deviations
    for (const auto& point : intersection_cloud_->points) {
        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_distances(1);
        if (kdtree.nearestKSearch(point, 1, nearest_indices, nearest_distances) > 0) {

            float deviation = std::sqrt(nearest_distances[0]);

            // Debugging: Print information to validate calculations
            std::cout << "Point from intersection_cloud_: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            pcl::PointXYZ nearest_point = nominal_cloud->points[nearest_indices[0]];
            std::cout << "Nearest point from nominal_cloud: (" << nearest_point.x << ", " << nearest_point.y << ", " << nearest_point.z << ")" << std::endl;
            std::cout << "Squared distance: " << nearest_distances[0] << std::endl;
            std::cout << "Deviation (sqrt of distance): " << deviation << std::endl;

      
            deviations->InsertNextValue(deviation);
    
        } else {

        std::cout << "No point found for: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

        }

    }


    vtkSmartPointer<vtkPolyData> polydata = convertToVtkPolyData(nominal_mesh);
    polydata->GetPointData()->SetScalars(deviations);
    viewer->addModelFromPolyData(polydata, "nominal_mesh");


    // HEAT MAP COLOR FORMATTING
    //Get the min-max of the values to be mapped
    double range[2];
    polydata->GetPointData()->GetScalars()->GetRange(range);
    
    double range_min = range[0] * 1;
    double range_max = range[1] * 1;
    // Create the lookup table and populate it
    vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetRange(range_min, range_max); 
    lut->SetHueRange(0.67, 0);  // flip the color range
    lut->SetScaleToLinear();
    
    // Assuming `viewer` is of type `pcl::visualization::PCLVisualizer::Ptr`
    viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  
    // This is the tricky part, as PCLVisualizer doesn't expose the mapper directly.
    // We're working with assumptions here. Please test if this actually works.
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polydata);
    mapper->SetLookupTable(lut);
    mapper->ScalarVisibilityOn();
    mapper->SetInterpolateScalarsBeforeMapping(1);
    mapper->UseLookupTableScalarRangeOff();
    mapper->SetScalarRange(lut->GetRange());
    mapper->Update();

    // // Here, we're setting the custom lookup table for the first actor. 
    // // If you have more, you'll need to adjust this.
    viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->SetMapper(mapper);


    // Create a scalar bar actor
    vtkSmartPointer<vtkScalarBarActor> scalarBar = vtkSmartPointer<vtkScalarBarActor>::New();
    scalarBar->SetLookupTable(lut);
    scalarBar->SetTitle("Deviations");
    scalarBar->SetNumberOfLabels(5);

    // Modify font for the labels
    vtkTextProperty* labelProperty = scalarBar->GetLabelTextProperty();
    labelProperty->SetFontFamilyToCourier();
    labelProperty->ItalicOff();
    labelProperty->BoldOff();  
    labelProperty->SetFontSize(2);

    // Modify font for the title
    vtkTextProperty* titleProperty = scalarBar->GetTitleTextProperty();
    titleProperty->SetFontFamilyToCourier();
    titleProperty->ItalicOff();
    titleProperty->BoldOff(); 
    titleProperty->SetFontSize(4);


 

    scalarBar->Modified();

    // Access internal VTK Renderer from PCLVisualizer
    // NOTE: This is risky and may break in future PCL versions
    vtkRenderer* renderer = viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer();

    // Add scalar bar actor to renderer
    if (renderer) {
        renderer->AddActor2D(scalarBar);
    }

    while (!viewer->wasStopped()) {
        
        viewer->spinOnce(100);

    }


    res.success = true;
    res.message = "Heatmap generated successfully.";
    return true;


}




    pcl::PointCloud<pcl::PointXYZ>::Ptr createSurfaceBoxPointCloud(float x_size, float y_size, float z_size, float resolution)
{



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Loop through the dimensions of the box to create surface points.
    for (float x = -x_size/2; x <= x_size/2; x += resolution)
    {
        for (float y = -y_size/2; y <= y_size/2; y += resolution)
        {
            cloud->points.push_back(pcl::PointXYZ(x, y, -z_size/2 + 0.25));  // Bottom face
            cloud->points.push_back(pcl::PointXYZ(x, y, z_size/2 + 0.25));   // Top face
        }
    }

    for (float x = -x_size/2; x <= x_size/2; x += resolution)
    {
        for (float z = -z_size/2; z <= z_size/2; z += resolution)
        {
            cloud->points.push_back(pcl::PointXYZ(x, -y_size/2, z + 0.25));  // Front face
            cloud->points.push_back(pcl::PointXYZ(x, y_size/2, z + 0.25));   // Back face
        }
    }

    for (float y = -y_size/2; y <= y_size/2; y += resolution)
    {
        for (float z = -z_size/2; z <= z_size/2; z += resolution)
        {
            cloud->points.push_back(pcl::PointXYZ(-x_size/2, y, z + 0.25));  // Left face
            cloud->points.push_back(pcl::PointXYZ(x_size/2, y, z + 0.25));   // Right face
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

 

    
    return cloud;
}



    void dynamicReconfigCallback(depth_processing::FiltersConfig &config, uint32_t level) {
    voxel_filter_leaf_size_ = config.voxel_leaf_size;
    sor_mean_k_ = config.mean_k;        
    sor_std_dev_ = config.std_dev;       
    passthrough_limit_min_ = config.passthrough_limit_min;
    passthrough_limit_max_ = config.passthrough_limit_max;

    max_iterations_ = config.max_iterations;
    max_correspondence_distance_ = config.max_correspondence_distance;
    transformation_epsilon_ = config.transformation_epsilon;
    euclidean_fitness_epsilon_ = config.euclidean_fitness_epsilon;
    ransac_outlier_rejection_threshold_ = config.ransac_outlier_rejection_threshold;
    use_reciprocal_correspondences_ = config.use_reciprocal_correspondences;

    // New variables for meshing
    normal_estimation_k_search_ = config.normal_estimation_k_search;
    search_radius_ = config.search_radius;
    mu_ = config.mu;
    maximum_nearest_neighbors_ = config.maximum_nearest_neighbors;
    maximum_surface_angle_ = config.maximum_surface_angle;
    minimum_angle_ = config.minimum_angle;
    maximum_angle_ = config.maximum_angle;
    normal_consistency_ = config.normal_consistency;

    poisson_depth_ = config.poisson_depth;
    poisson_scale_ = config.poisson_scale;
    poisson_solver_divide_ = config.poisson_solver_divide;
    poisson_iso_divide_ = config.poisson_iso_divide;
    poisson_confidence_ = config.poisson_confidence;
    poisson_point_weight_ = config.poisson_point_weight;
    poisson_samples_per_node_ = config.poisson_samples_per_node;
    poisson_manifold_ = config.poisson_manifold;


    colormap_min_ = config.colormap_min;
    colormap_max_ = config.colormap_max;

}

// This is meant to run the viewer at a separate thread
// TODO investigate possibility of running this on a separate thread
void runViewer()
{

    viewer->registerKeyboardCallback(&DepthProcessor::keyboardEventOccurred, *this);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
       
    }
}

// This captures keyboard events for the PCL visualizer.
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* nothing)
{
    ROS_INFO("key pressed on visualization window");

    if (event.getKeySym() == "q" && event.keyDown())
    {
        close_viewer = true;
    }
}




private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber depth_image_sub_;

    ros::Publisher cloud_pub_;  
    ros::Publisher adjusted_cloud_pub_;
    ros::Publisher retrieved_cloud_pub_;
    ros::Publisher combined_cloud_pub_;
    ros::Publisher aligned_cloud_pub_;
    ros::Publisher reference_cloud_pub_;
    ros::Publisher intersection_pub_;

    ros::ServiceServer capture_service_;
    ros::ServiceServer retrieve_service_;
    ros::ServiceServer align_service_;
    ros::ServiceServer publish_all_service_; //rename
    ros::ServiceServer intersect_service_; //rename
    ros::ServiceServer mesh_service_;
    ros::ServiceServer deviation_service_; //rename
    ros::ServiceServer load_service_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr adjusted_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> measurements_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_registered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_cloud_;

    std::vector<tf::StampedTransform> transforms_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr nominal_cloud;
   

    // std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, tf::StampedTransform>> measurements_;

    std::vector<Eigen::Matrix4f> poses_;

    tf::TransformListener* tf_listener_;
    tf::TransformListener listener; 
    
    pcl::PolygonMesh nominal_mesh;
    pcl::PolygonMesh scanned_mesh;
    
    std::string stl_file_path;


     // Filter flags
    bool use_voxel_filter_;
    bool use_sor_filter_;
    bool use_passthrough_filter_;
    bool use_sac_registration_;


    //GUI controls
    dynamic_reconfigure::Server<depth_processing::FiltersConfig> server_;
    dynamic_reconfigure::Server<depth_processing::FiltersConfig>::CallbackType f_;

    double voxel_filter_leaf_size_;
    int sor_mean_k_;
    double sor_std_dev_;
    double passthrough_limit_min_;
    double passthrough_limit_max_;

    int max_iterations_;
    double max_correspondence_distance_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    double ransac_outlier_rejection_threshold_;
    bool use_reciprocal_correspondences_;

    // Normal Estimation
    int normal_estimation_k_search_;

    // GreedyProjectionTriangulation
    double search_radius_;
    double mu_;
    int maximum_nearest_neighbors_;
    double maximum_surface_angle_;
    double minimum_angle_;
    double maximum_angle_;
    bool normal_consistency_;

    // Poisson
    int poisson_depth_;
    double poisson_scale_;
    int poisson_solver_divide_;
    int poisson_iso_divide_;
    bool poisson_confidence_;
    double poisson_point_weight_;
    double poisson_samples_per_node_;
    bool poisson_manifold_;

    double colormap_min_;
    double colormap_max_;
    //FOR TESTING PURPOSES ONLY. MUST REMOVE
    float x_size = 0.5;
    float y_size = 0.5;
    float z_size = 0.5;
    float resolution = 0.01;

    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool close_viewer = false;



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_processor");
    DepthProcessor dp;
    ros::spin();
    return 0;
}
