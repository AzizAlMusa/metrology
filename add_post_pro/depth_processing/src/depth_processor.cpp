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
    if (current_cloud_) //current_cloud_
    {
        measurements_.push_back(current_cloud_); //current_cloud_

        // Capture the pose
        tf::StampedTransform transform;
        try
        {
            // Assuming the transformation is from a "world" frame to the "camera_link" frame.
            // Adjust the frame names as per your setup.
            tf_listener_->lookupTransform("world", "camera_link", ros::Time(0), transform);
         
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failed to get camera pose: %s", ex.what());
            res.success = false;
            res.message = "Failed to get camera pose.";
            return true;
        }

        // Convert tf::StampedTransform to Eigen::Matrix4f
        Eigen::Matrix4f eigen_transform;
        Eigen::Translation3f trans(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        Eigen::Quaternionf quat(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
        eigen_transform = (trans * quat).matrix();



        // Store the captured pose
        poses_.push_back(eigen_transform);

        res.success = true;
        res.message = "Measurement and pose captured successfully.";
        return true;
    }
    res.success = false;
    res.message = "No current cloud available to capture.";
    return true;
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

    bool publishAllMeasurements(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if (measurements_.empty())
        {
            res.success = false;
            res.message = "No measurements available.";
            return true;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& cloud : measurements_)
        {
            *combined_cloud += *cloud;
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*combined_cloud, output);
        output.header.frame_id = "world";  // Assuming "world" is your global frame
        combined_cloud_pub_.publish(output);

        res.success = true;
        res.message = "Published all measurements combined.";
        return true;
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

    // Apply a 90-degree rotation around the y-axis to the point cloud
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(1.5708, Eigen::Vector3f::UnitY())); // 1.5708 radians = 90 degrees
    transform.rotate(Eigen::AngleAxisf(-1.5708, Eigen::Vector3f::UnitZ())); // -1.5708 radians = -90 degrees for x-axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 adjusted_output;
    pcl::toROSMsg(*transformed_cloud, adjusted_output);
    adjusted_output.header = cloud_msg->header;  // Use the same header as the input cloud for consistency
    adjusted_cloud_ = transformed_cloud;
    adjusted_cloud_pub_.publish(adjusted_output);

    // ROS_INFO("Received point cloud with %lu points, filtered to %lu points", cloud->points.size(), cloud_filtered->points.size());
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
    std::vector<Eigen::Matrix4f> poses_;

    tf::TransformListener* tf_listener_;

 
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
