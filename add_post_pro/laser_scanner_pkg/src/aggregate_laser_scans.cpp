#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



ros::Publisher pub;
// pcl::PointCloud<pcl::PointXYZ>::Ptr big_cloud(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 big_cloud;
bool is_moving =  true;
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
  // Convert the ROS PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_big_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::fromROSMsg(*input_cloud, *input_pcl_cloud);
  
// Publish the big point cloud
  sensor_msgs::PointCloud2 output_cloud;
  // Concatenate the point clouds
  if (true) {
     pcl::concatenatePointCloud(big_cloud, *input_cloud, output_cloud);
  

    pcl::fromROSMsg(output_cloud, *pcl_big_cloud);

    pcl::toROSMsg(*pcl_big_cloud, big_cloud);
  } else {
     output_cloud = big_cloud;
  }
 
  
  std::cout << output_cloud.row_step << std::endl;
 
//   pcl::toROSMsg(*big_cloud, output_cloud);
//   std::cout << output_cloud << std::endl;
  pub.publish(output_cloud);
}

void checkMoving(const sensor_msgs::JointStateConstPtr& joint_states){
    //This looks at joint angle velocities to determine if the robot is moving or not
    bool all_less_than_threshold = true;
    float max_velocity = 1e-03;
    for (int i = 0; i < 7; i++) {

        bool velocity_check = joint_states->velocity[i] >= max_velocity;
        all_less_than_threshold *= velocity_check;
    // if (joint_states->velocity[i] >= max_velocity) {
         
    //         all_less_than_threshold = false;
    //         break;
    //     }
    //     else {
    //          all_less_than_threshold = true;
    //     }
    }

    is_moving = all_less_than_threshold;
    std::cout << is_moving <<std::endl;

}   

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "concatenate_pointclouds");

  // Create a node handle
  ros::NodeHandle nh;

  // Subscribe to the input point cloud topic
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/transformed_cloud", 1, cloudCallback);
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, checkMoving);

  // Advertise the output point cloud topic
  pub = nh.advertise<sensor_msgs::PointCloud2>("/output_cloud_topic", 1);

  // Spin
  ros::spin();

  return 0;
}
