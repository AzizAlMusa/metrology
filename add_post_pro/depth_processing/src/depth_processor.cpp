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

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/transforms.h>


class DepthProcessor
{
public:
      DepthProcessor() : nh_("~")
    {
        cloud_sub_ = nh_.subscribe("/robot/depth_camera/points", 1, &DepthProcessor::pointCloudCallback, this);
        depth_image_sub_ = nh_.subscribe("/robot/depth_camera/image_raw", 1, &DepthProcessor::depthImageCallback, this);

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
        adjusted_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("adjusted_cloud", 1);
        retrieved_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("retrieved_cloud", 1);
        combined_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);

        capture_service_ = nh_.advertiseService("capture_measurement", &DepthProcessor::captureMeasurement, this);
        retrieve_service_ = nh_.advertiseService("get_measurement", &DepthProcessor::getMeasurement, this);
        align_service_ = nh_.advertiseService("align_measurements", &DepthProcessor::alignPointClouds, this);
        publish_all_service_ = nh_.advertiseService("publish_all_measurements", &DepthProcessor::publishAllMeasurements, this);

        

         // Load filter parameters
        nh_.param("use_voxel_filter", use_voxel_filter_, true);
        nh_.param("use_sor_filter", use_sor_filter_, true);
        nh_.param("use_passthrough_filter", use_passthrough_filter_, true);


        tf_listener_ = new tf::TransformListener();


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
        voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel_filter.filter(*cloud_filtered);
    }

    // Statistical Outlier Removal
    if (use_sor_filter_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
        sor_filter.setInputCloud(cloud_filtered);
        sor_filter.setMeanK(50);
        sor_filter.setStddevMulThresh(1.0);
        sor_filter.filter(*cloud_filtered);
    }

    // Passthrough Filter
    if (use_passthrough_filter_)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 2.0);
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
    if (measurements_.size() < 2)
    {
        res.success = false;
        res.message = "Not  enough point clouds for alignment.";
        return true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Assuming the first point cloud as reference
    *combined_cloud += *measurements_[0];

    for (size_t i = 1; i < measurements_.size(); i++)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(measurements_[i]);
        icp.setInputTarget(combined_cloud);
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud, poses_[i]);  // Using the stored pose for initial guess
        *combined_cloud += aligned_cloud;
    }

    // Store the combined cloud
    current_cloud_ = combined_cloud;

    res.success = true;
    res.message = "Point clouds aligned successfully.";
    return true;
}





private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber depth_image_sub_;

    ros::Publisher cloud_pub_;  
    ros::Publisher adjusted_cloud_pub_;
    ros::Publisher retrieved_cloud_pub_;
    ros::Publisher combined_cloud_pub_;

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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_processor");
    DepthProcessor dp;
    ros::spin();
    return 0;
}
