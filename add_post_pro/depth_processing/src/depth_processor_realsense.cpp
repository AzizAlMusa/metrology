#include <ros/ros.h>

#include <sensor_msgs/Image.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>


#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>


#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/transforms.h>

#include <dynamic_reconfigure/server.h>
#include <depth_processing/FiltersConfig.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <chrono>



class DepthProcessor
{
public:
     
      DepthProcessor() : nh_("~")
    {   


        


        cloud_sub_ = nh_.subscribe("/camera/depth/color/points", 1, &DepthProcessor::pointCloudCallback, this);
        depth_image_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &DepthProcessor::depthImageCallback, this);

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
        adjusted_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("adjusted_cloud", 1);
        retrieved_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("retrieved_cloud", 1);
        combined_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
        aligned_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
        reference_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reference_cloud", 1);

        capture_service_ = nh_.advertiseService("capture_measurement", &DepthProcessor::captureMeasurement, this);
        retrieve_service_ = nh_.advertiseService("get_measurement", &DepthProcessor::getMeasurement, this);
        align_service_ = nh_.advertiseService("align_measurements", &DepthProcessor::alignPointClouds, this);
        publish_all_service_ = nh_.advertiseService("publish_all_measurements", &DepthProcessor::publishAllMeasurements, this);

        

         // Load filter parameters
        nh_.param("use_voxel_filter", use_voxel_filter_, true);
        nh_.param("use_sor_filter", use_sor_filter_, true);
        nh_.param("use_passthrough_filter", use_passthrough_filter_, true);
        nh_.param("use_sac_registration", use_sac_registration_, true);
        
        tf_listener_ = new tf::TransformListener();


        f_ = boost::bind(&DepthProcessor::dynamicReconfigCallback, this, _1, _2);
        server_.setCallback(f_);

        surface_cloud = createSurfaceBoxPointCloud(x_size, y_size, z_size, resolution);

       
         // Initialize viewer
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);  // The parameter specifies the size of the axes

        // Run the viewer on a separate thread
        // std::thread viewerThread(&DepthProcessor::runViewer, this);
        // viewerThread.detach();


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
    ROS_INFO("Capture Measurement Transform");
     if (current_cloud_) // Check if there's any data
    {   
        tf::StampedTransform transform;
        try
        {
            ROS_INFO("Calling Lookup transform");
            tf_listener_->lookupTransform("world", "camera_depth_optical_frame", ros::Time(0), transform);
            ROS_INFO("Lookup transform Done");
            measurements_.push_back(current_cloud_);  // Store the transformed point cloud
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
        ROS_INFO("Get Measurement Service Called");
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
     ROS_INFO("Publish Measurements Called");
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
    // ROS_INFO("Received depth image with width: %d and height: %d", image_msg->width, image_msg->height);
    // TODO: Any direct processing on the depth image if required
    // int x = 1
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
    // Change frame ID of the filtered pointcloud
    output.header.frame_id = "camera_depth_optical_frame";
    
    current_cloud_ = cloud_filtered;
    cloud_pub_.publish(output);


    
    // // Adjust the point cloud based on the transform between camera_link and camera_link_adjusted

    // tf::TransformListener listener;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
    // tf::StampedTransform transform;
    // try
    // {
    //     listener.waitForTransform("camera_link", "camera_link_adjusted", ros::Time(0), ros::Duration(3.0));
    //     listener.lookupTransform("camera_link", "camera_link_adjusted", ros::Time(0), transform);
    //     pcl_ros::transformPointCloud(*cloud_filtered, *cloud_adjusted, transform);
    // }
    // catch (tf::TransformException& ex)
    // {
    //     ROS_ERROR("Transform error: %s", ex.what());
    //     return;
    // }

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
    pcl::toROSMsg(*surface_cloud, ros_msg);
    
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

    icp.setMaximumIterations(200);

    // This extremely important for good quality registrations. The nature of the problem means measurements are not that far off.
    // a 2 cm correspondence should be generous and avoids misaligned registrations.
    icp.setMaxCorrespondenceDistance(0.02); 

    icp.setTransformationEpsilon(1e-8);

    viewer->addPointCloud<pcl::PointXYZ>(surface_cloud, "surface_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 1.0, "surface_cloud");  // Light Blue: R=0.5, G=0.5, B=1.0
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
        icp.setInputTarget(surface_cloud);
        icp.align(*registered_cloud);

        if (icp.hasConverged())
        {
            // Add the aligned point cloud to the final cloud
            *final_registered_cloud += *registered_cloud;

            // Update the viewer
            // viewer->removeAllPointClouds();
            viewer->removePointCloud("registered_cloud");
            viewer->removePointCloud("transformed_cloud");

            viewer->addPointCloud<pcl::PointXYZ>(registered_cloud, "registered_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0.5, "registered_cloud");  // Light Blue: R=0.5, G=0.5, B=1.0

            
            viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "transformed_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "transformed_cloud");  // Purple: R=0.5, G=0, B=0.5
   

            viewer->spinOnce(5000);

            viewer->removePointCloud("registered_cloud");
            viewer->removePointCloud("transformed_cloud");
  

        }
        else

        {
            ROS_WARN("ICP failed to converge for measurement %zu.", i);
        }


    }

    viewer->addPointCloud<pcl::PointXYZ>(final_registered_cloud, "fina_registered_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "fina_registered_cloud");  // Purple: R=0.5, G=0, B=0.5
   

    viewer->spinOnce(10000);

    // Publishing the combined registered cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*final_registered_cloud, output);
    output.header.frame_id = "world";
    aligned_cloud_pub_.publish(output);

    res.success = true;
    res.message = "Aligned and published the combined measurements.";
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

    ros::ServiceServer capture_service_;
    ros::ServiceServer retrieve_service_;
    ros::ServiceServer align_service_;
    ros::ServiceServer publish_all_service_;


    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr adjusted_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> measurements_;

    std::vector<tf::StampedTransform> transforms_;

    // std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, tf::StampedTransform>> measurements_;

    std::vector<Eigen::Matrix4f> poses_;

    tf::TransformListener* tf_listener_;
    tf::TransformListener tf_listener_capture_;
    tf::TransformListener listener; 
 
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
