#include<ros/ros.h>
#include<ros/console.h>
#include<math.h>
#include<pcl/common/io.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <pcl_sensor_data/ProcessPointcloud.h>
#include <pcl_sensor_data/StartRecording.h>
#include <pcl_sensor_data/StopRecording.h>
#include <pcl_sensor_data/RestartRecording.h>

class LaserProcessor
{

    private:
        ros::NodeHandle nh, pnh;
        ros::Subscriber pcl_sub;
        ros::Publisher result_pub;
        ros::ServiceServer start_srv, stop_srv, restart_srv, process_srv;
        tf::Transformer transformer;
        tf::TransformListener listener;
        tf::StampedTransform stamped_transform;

        std::string frame_id;
		unsigned int seq;

        std::vector<uint8_t> sum_data;
        sensor_msgs::PointCloud2 cumulative_pointcloud;
        sensor_msgs::PointCloud2 transformed;

    public:
        LaserProcessor() : nh("laser_processor"), pnh("~"), seq(0)
        {
            ROS_INFO("Starting laser processing services");
            std::string start_srv_name = this->pnh.param<std::string>("start_service_name", "start_recording");	
			this->start_srv  = this->nh.advertiseService(start_srv_name, &LaserProcessor::start_recording, this);
            ROS_INFO("Initialized start service");

            std::string stop_srv_name = this->pnh.param<std::string>("stop_service_name", "stop_recording");	
			this->stop_srv  = this->nh.advertiseService(stop_srv_name, &LaserProcessor::stop_recording, this);
            ROS_INFO("Initialized stop service");

            std::string restart_srv_name = this->pnh.param<std::string>("restart_service_name", "restart_recording");		
			this->restart_srv  = this->nh.advertiseService(restart_srv_name, &LaserProcessor::restart_recording, this);
            ROS_INFO("Initialized restart service");

            std::string process_srv_name = this->pnh.param<std::string>("process_service_name", "process_pointcloud");		
			this->process_srv  = this->nh.advertiseService(process_srv_name, &LaserProcessor::process_pointcloud, this);
            ROS_INFO("Initialized process service");

            this->frame_id = this->pnh.param<std::string>("scanner_frame_id", "scancontrol_aperture_frame");

        }
        
        bool restart_recording(pcl_sensor_data::RestartRecording::Request &req,
                             pcl_sensor_data::RestartRecording::Response &res)
        {
            std::string pointcloud_topic_name = this->pnh.param<std::string>("scanner_pointcloud_topic", "/scancontrol_pointcloud");
            this->pcl_sub = nh.subscribe(pointcloud_topic_name, 150, &LaserProcessor::pcl_callback, this);
            ROS_INFO("Restarted recording");
            res.success = true;
            return true;
        }

        bool start_recording(pcl_sensor_data::StartRecording::Request &req,
                             pcl_sensor_data::StartRecording::Response &res)
        {
            std::string pointcloud_topic_name = this->pnh.param<std::string>("scanner_pointcloud_topic", "/scancontrol_pointcloud");
            this->pcl_sub = nh.subscribe(pointcloud_topic_name, 10, &LaserProcessor::pcl_callback, this);
            this->sum_data.clear();
            ROS_INFO("Started recording");
            res.success = true;
            return true;
        }

        bool stop_recording(pcl_sensor_data::StopRecording::Request &req,
                            pcl_sensor_data::StopRecording::Response &res)
        {
            this->pcl_sub.shutdown();
            ROS_INFO("Stopped recording");
            res.success = true;
            return true;
        }

        bool process_pointcloud(pcl_sensor_data::ProcessPointcloud::Request &req,
                                pcl_sensor_data::ProcessPointcloud::Response &res)
        {
            this->cumulative_pointcloud.header = transformed.header;
            this->cumulative_pointcloud.header.stamp = ros::Time(0);
            this->cumulative_pointcloud.data = this->sum_data;
            this->cumulative_pointcloud.height = 1;
            this->cumulative_pointcloud.point_step = transformed.point_step;
            this->cumulative_pointcloud.width = sum_data.size()/cumulative_pointcloud.point_step;
            this->cumulative_pointcloud.is_dense = true;
            this->cumulative_pointcloud.row_step = this->sum_data.size();
            this->cumulative_pointcloud.fields = transformed.fields;
            ros::Rate loop_rate(10);
            this->result_pub = pnh.advertise<sensor_msgs::PointCloud2>("results", 1000);
            ROS_INFO("Preparing to publish result");
            // TODO: Remove this loop
            while (true)
            {
                this->cumulative_pointcloud.header.stamp = ros::Time(0);
                this->result_pub.publish(this->cumulative_pointcloud);
                loop_rate.sleep();
                ros::spinOnce();
            }

            res.success = true;
            return true;
        }

        void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &scan_in)
        {
            sensor_msgs::PointCloud2 transformed;
            sensor_msgs::PointCloud2 res;
            ros::Time time = ros::Time(0);
            try{
                this->listener.waitForTransform("/world", this->frame_id, scan_in->header.stamp, ros::Duration(0.1));
                this->listener.lookupTransform("/world", this->frame_id,  scan_in->header.stamp, stamped_transform);
            } catch(tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
                ROS_ERROR("RIP");
                return;
            }
            pcl_ros::transformPointCloud("world", *scan_in, this->transformed, this->listener);

            this->sum_data.reserve(this->sum_data.size() + this->transformed.data.size());
            this->sum_data.insert(this->sum_data.end(), this->transformed.data.begin(), this->transformed.data.end());
        }
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_processing_services");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	LaserProcessor laser_processor;

	ros::waitForShutdown();
	spinner.stop();
	return 0;
}