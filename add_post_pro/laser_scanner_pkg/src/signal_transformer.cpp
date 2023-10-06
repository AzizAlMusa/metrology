#include <ros/ros.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_node");
  ros::NodeHandle nh;

  // Create a TransformListener to listen for transform updates
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  std::string laser_frame = "true_laser"; 
  std::string target_frame = "world"; 


  // Create a publisher for the transformed point cloud
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 10);

  // Create a subscriber for the input point cloud
//   ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("pointcloud_topic", 10,
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("scancontrol_pointcloud", 10,
      [&](const sensor_msgs::PointCloud2::ConstPtr& input_pc2) {
            
            // Create a container for the output pointcloud
            sensor_msgs::PointCloud2 output_pc2;
        
            output_pc2.header.frame_id = target_frame;

     

            // Create a TransformListener to listen for transform updates
             tf::TransformListener listener;

            // Wait for the transform from the input pointcloud frame to the output frame to become available
            tf::StampedTransform transform;

            std::vector<std::string> frame_ids;
            listener.getFrameStrings(frame_ids);

            // Print the list of frame IDs
            for (const auto& frame_id : frame_ids) {
                ROS_INFO_STREAM("Frame ID: " << frame_id);
            }

            try
            {   
                
                // listener.waitForTransform(input_pc2->header.frame_id,  output_pc2.header.frame_id, ros::Time(0), ros::Duration(0.1));
                // listener.lookupTransform(input_pc2->header.frame_id,  output_pc2.header.frame_id, ros::Time(0), transform);
                listener.waitForTransform(output_pc2.header.frame_id, input_pc2->header.frame_id, ros::Time(0), ros::Duration(0.1));
                listener.lookupTransform(output_pc2.header.frame_id, input_pc2->header.frame_id, ros::Time(0), transform);
            }
            catch (tf::TransformException& ex)
            {
                ROS_WARN_STREAM("Could not get transform from " << input_pc2->header.frame_id << " to " << output_pc2.header.frame_id << ": " << ex.what());
                return;
            }

            // Convert the input pointcloud to a PCL pointcloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*input_pc2, *input_cloud);

            // Transform the pointcloud using pcl_ros::transformPointCloud()
            pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl_ros::transformPointCloud(*input_cloud, *output_cloud, transform);

            // Convert the output pointcloud to a sensor_msgs::PointCloud2 message
            pcl::toROSMsg(*output_cloud, output_pc2);
            output_pc2.header.stamp = ros::Time::now();
            output_pc2.header.frame_id = target_frame;

            // Publish the output pointcloud
            pub.publish(output_pc2);

         } );

  ros::spin();
  return 0;
}
