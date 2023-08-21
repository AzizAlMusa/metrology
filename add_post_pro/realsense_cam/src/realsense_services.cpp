/*
 * Copyright @ CERLAB
 * This node provides services to access RealSense D415/D430 topics.
 */

#include <iostream>
#include <unistd.h>
#include <ctime>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <realsense_cam/FetchOneDepth.h>
#include <realsense_cam/FetchOnePointCloud.h>
#include <realsense_cam/FetchOneRGB.h>
#include <realsense_cam/SaveOnePCL.h>
#include <realsense_cam/SaveOneRGB.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread/mutex.hpp>		// Used to lock resources when obtaining picture frames, maybe not necessary
#include <boost/thread/shared_mutex.hpp>

#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>

/*
	DONE: 
		-Obtain single pointcloud
		-Obtain single depth image
*/

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class RealSenseCam
{
	private:

		ros::NodeHandle nh, pnh;		// pnh = parameter node handle 
		ros::Subscriber rs_pointcloud_frame, rs_depth_frame, rs_rgb_frame;
		ros::Publisher rs_pointcloud_discrete, rs_depth_discrete, rs_rgb_discrete;
		ros::ServiceServer next_depth, next_pointcloud, convert_depth_to_pointcloud, next_rgb, save_pcl, save_rgb;

		std::string frame_id, dir;
		unsigned int seq;

		sensor_msgs::PointCloud2Ptr previous_pointcloud;
		sensor_msgs::ImagePtr previous_depth;	// Holds the most recently updated frames from the pointcloud and depth topics
		sensor_msgs::ImagePtr previous_rgb;
		boost::shared_mutex update_frame_lock;		// Used to lock resources when updating the frames the node is holding

	public:
		RealSenseCam() : nh("realsense_services"), pnh("~"), frame_id("cam"), seq(0)
		{
			ROS_INFO("[realsense_services] Initializing...");

			// Pointcloud retrieval services
			std::string pointcloud_topic_name = this->pnh.param<std::string>("realsense_pointcloud_topic", "/D415/depth/color/points"); //Return value from parameter "realsense_pointcloud_topic" if available
																																		  //If not, default to second argument
			this->rs_pointcloud_frame = this->nh.subscribe(pointcloud_topic_name, 20, &RealSenseCam::rs_pointcloud_callback, this);		  //Subscribe to retrieve pointcloud messages from camera
																																		  //and store in this node	

			std::string pc_pub_topic = this->pnh.param<std::string>("pointcloud_pub_topic", "pointcloud");	//Get string name from parameters or use default
			this->rs_pointcloud_discrete = this->nh.advertise<sensor_msgs::PointCloud2>(pc_pub_topic, 20);		//Advertise pointcloud topic

			std::string pc_srv_name = this->pnh.param<std::string>("pc_service_name", "next_pointcloud");		//Get string name from parameters or use default
			this->next_pointcloud  = this->nh.advertiseService(pc_srv_name, &RealSenseCam::return_pointcloud, this);

			// Depth retrieval services
			std::string depth_topic_name = this->pnh.param<std::string>("realsense_depth_topic", "/D415/depth/image_rect_raw");

			this->rs_depth_frame = this->nh.subscribe(depth_topic_name, 20, &RealSenseCam::rs_depth_callback, this);		//Subscribe to depth image topic and keep updating the depth image this 
																															//node stores
			std::string depth_pub_topic = this->pnh.param<std::string>("depth_pub_topic", "depth_frame");
			this->rs_depth_discrete = this->nh.advertise<sensor_msgs::Image>(depth_pub_topic, 20);

			std::string depth_srv_name = this->pnh.param<std::string>("depth_service_name", "next_depth");
			this->next_depth = this->nh.advertiseService(depth_srv_name, &RealSenseCam::return_depth, this);

			// RGB retrieval services
			std::string rgb_topic_name = this->pnh.param<std::string>("realsense_rgb_topic", "/D415/color/image_raw");

			this->rs_rgb_frame = this->nh.subscribe(rgb_topic_name, 20, &RealSenseCam::rs_rgb_callback, this);		//Subscribe to rgb image topic and keep updating the rgb image this 
																															//node stores
			std::string rgb_pub_topic = this->pnh.param<std::string>("rgb_pub_topic", "rgb_frame");
			this->rs_rgb_discrete = this->nh.advertise<sensor_msgs::Image>(rgb_pub_topic, 20);

			std::string rgb_srv_name = this->pnh.param<std::string>("rgb_service_name", "next_rgb");
			this->next_rgb = this->nh.advertiseService(rgb_srv_name, &RealSenseCam::return_rgb, this);

			ROS_INFO("[realsense_cam] Done initializing depth, rgb, and pointcloud fetching services");				//Update with newest services

			// PCL Saving Service
			std::string pcl_save_name = this->pnh.param<std::string>("pcl_save_service_name", "save_pcl");
			std::string pcl_save_dir = this->pnh.param<std::string>("pcl_save_dir_name", "/home/depowdering/src/realsense_cam/data/");
			this->dir = pcl_save_dir;
			this->save_pcl = this->nh.advertiseService(pcl_save_name, &RealSenseCam::write_pcl, this);

			// PCL Saving Service
			std::string rgb_save_name = this->pnh.param<std::string>("rgb_save_service_name", "save_rgb");
			std::string rgb_save_dir = this->pnh.param<std::string>("rgb_save_dir_name", "/home/depowdering/src/realsense_cam/data/");
			this->save_rgb = this->nh.advertiseService(rgb_save_name, &RealSenseCam::write_rgb, this);
		}

	private:
		
		void rs_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& frame)
		{
			// ROS_INFO("In pointcloud callback");
			boost::upgrade_lock<boost::shared_mutex> _lock(this->update_frame_lock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(_lock);
			this->previous_pointcloud = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2(*frame));
		}

		void rs_depth_callback(const sensor_msgs::ImageConstPtr& frame)
		{
			// ROS_INFO("In depth callback");
			boost::upgrade_lock<boost::shared_mutex> _lock(this->update_frame_lock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(_lock);
			this->previous_depth = sensor_msgs::ImagePtr(new sensor_msgs::Image(*frame));
		}

		void rs_rgb_callback(const sensor_msgs::ImageConstPtr& frame)
		{
			// ROS_INFO("In RGB callback");
			boost::upgrade_lock<boost::shared_mutex> _lock(this->update_frame_lock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(_lock);
			this->previous_rgb = sensor_msgs::ImagePtr(new sensor_msgs::Image(*frame));
		}

		bool return_pointcloud(realsense_cam::FetchOnePointCloud::Request &req,
								realsense_cam::FetchOnePointCloud::Response &res)
		{
			ROS_INFO("[realsense_cam] Sending pointcloud");
			res.success = true;
			res.reason = "Success!";
			res.pointcloud = *this->previous_pointcloud;
			return true;
		}

		bool return_depth(realsense_cam::FetchOneDepth::Request &req,
							realsense_cam::FetchOneDepth::Response &res)
		{
			
			ROS_INFO("[realsense_cam] Sending depth image");
			res.success = true;
			res.reason = "Success!";
			res.depth = *this->previous_depth;
			return true;
		}
		bool return_rgb(realsense_cam::FetchOneRGB::Request &req,
							realsense_cam::FetchOneRGB::Response &res)
		{
			
			ROS_INFO("[realsense_cam] Sending RGB image");
			res.success = true;
			ROS_INFO("[realsense_cam] Set success");
			res.reason = "Success!";
			ROS_INFO("[realsense_cam] Set reason");
			res.rgb = *this->previous_rgb;
			ROS_INFO("[realsense_cam] Leaving RGB return function");
			return true;
		}

		bool write_pcl(realsense_cam::SaveOnePCL::Request &req,
						realsense_cam::SaveOnePCL::Response &res)
		{
			ROS_INFO("[realsense_cam] Attempting to write Pointcloud image");
			pcl::PCLPointCloud2 pcl_pc2;
			sensor_msgs::PointCloud2ConstPtr curr_pcl(this->previous_pointcloud);
			pcl_conversions::toPCL(*curr_pcl, pcl_pc2);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
			
			// Time stamp the pcd file
			std::time_t now = std::time(NULL);
			std::tm * ptm = std::localtime(&now);
			char buffer[32];
			std::strftime(buffer, 32, "%H%M%S", ptm);
			pcl::io::savePCDFileASCII(dir + "/pcd/test_pcd" + buffer + ".pcd", *temp_cloud);
			res.success = true;
			ROS_INFO("[realsense_cam] Successfully wrote to pcd file");
			return true;
			
		}

		bool write_rgb(realsense_cam::SaveOneRGB::Request &req,
						realsense_cam::SaveOneRGB::Response &res)
		{
			ROS_INFO("[realsense_cam] Attempting to write RGB image");
			cv_bridge::CvImagePtr cv_ptr;
			sensor_msgs::ImageConstPtr cnst_img(this->previous_rgb);
			try {
				cv_ptr = cv_bridge::toCvCopy(cnst_img);
			} catch (cv_bridge::Exception& e) 
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				res.success = false;
				return false;
			}
			cv::Mat img = cv_ptr->image;
			//Time stamp the image
			std::time_t now = std::time(NULL);
			std::tm * ptm = std::localtime(&now);
			char buffer[32];
			std::strftime(buffer, 32, "%H%M%S", ptm);
			std::cout << "Writing to: " << dir+"rgb/test_rgb"+buffer+".png" << std::endl;
			cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
			// cv::imshow("image", img);
			// cv::waitKey(0);
			cv::imwrite(dir + "rgb/test_rgb"+buffer+".png", img);
			res.success = true;
			ROS_INFO("[realsense_cam] Successfully wrote to rgb file");
			return true;
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "realsense_services");
	ros::AsyncSpinner spinner(4);
	spinner.start();

	RealSenseCam rs_camera;

	ros::waitForShutdown();
	spinner.stop();

	return 0;
}