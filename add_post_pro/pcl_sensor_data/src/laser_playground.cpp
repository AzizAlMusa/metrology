#include<ros/ros.h>
#include<ros/console.h>
#include<math.h>
#include<pcl/common/io.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Point32.h>

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
#include <pcl_conversions/pcl_conversions.h>

//To save pcd file
#include <pcl/io/pcd_io.h>


#include <laser_geometry/laser_geometry.h>

class LaserProcessor
{

    private:
        ros::NodeHandle nh, pnh;
        ros::Subscriber pcl_sub;
        ros::Publisher result_pub;// = pnh.advertise<sensor_msgs::PointCloud>("results", 1000);
       
        ros::ServiceServer start_srv, stop_srv, restart_srv, process_srv;
        tf::Transformer transformer;
        tf::TransformListener listener;
        tf::TransformListener listener_;

        tf::StampedTransform stamped_transform;

        std::string frame_id = "/turntable_plate";
		unsigned int seq;

        std::vector<geometry_msgs::Point32> sum_data;
        sensor_msgs::PointCloud cumulative_pointcloud;
        sensor_msgs::PointCloud transformed;


        laser_geometry::LaserProjection projector_;

    public:
        LaserProcessor() : nh("laser_playground"), pnh("~"), seq(0)
        {
            ROS_INFO("Starting laser processing services");
            std::string start_srv_name = this->pnh.param<std::string>("start_service_name", "start_recording");	
			this->start_srv  = this->nh.advertiseService(start_srv_name, &LaserProcessor::start_recording, this);
            ROS_INFO("Initialized start service");

            std::string stop_srv_name = this->pnh.param<std::string>("stop_service_name", "stop_recording");	
			this->stop_srv  = this->nh.advertiseService(stop_srv_name, &LaserProcessor::stop_recording, this);
            ROS_INFO("Initialized stop service");

            // std::string restart_srv_name = this->pnh.param<std::string>("restart_service_name", "restart_recording");		
			// this->restart_srv  = this->nh.advertiseService(restart_srv_name, &LaserProcessor::restart_recording, this);
            // ROS_INFO("Initialized restart service");

            std::string process_srv_name = this->pnh.param<std::string>("process_service_name", "process_pointcloud");		
			this->process_srv  = this->nh.advertiseService(process_srv_name, &LaserProcessor::process_pointcloud, this);
            ROS_INFO("Initialized process service");

            //this->frame_id = this->pnh.param<std::string>("scanner_frame_id", "scancontrol_aperture_frame");
            
        }
        
        // bool restart_recording(pcl_sensor_data::RestartRecording::Request &req,
        //                      pcl_sensor_data::RestartRecording::Response &res)
        // {
        //     std::string pointcloud_topic_name = this->pnh.param<std::string>("scanner_pointcloud_topic", "/arm/laser_scan");
        //     this->pcl_sub = nh.subscribe(pointcloud_topic_name, 150, &LaserProcessor::pcl_callback, this);
        //     ROS_INFO("Restarted recording");
        //     res.success = true;
        //     return true;
        // }

        // bool start_recording(pcl_sensor_data::StartRecording::Request &req,
        //                      pcl_sensor_data::StartRecording::Response &res)
        // {
        //     std::string pointcloud_topic_name = this->pnh.param<std::string>("scanner_pointcloud_topic", "/arm/laser_scan");
        //     this->pcl_sub = nh.subscribe(pointcloud_topic_name, 10, &LaserProcessor::pcl_callback, this);
        //     this->sum_data.clear();
        //     ROS_INFO("Started recording");
        //     res.success = true;
        //     return true;
        // }

        // bool stop_recording(pcl_sensor_data::StopRecording::Request &req,
        //                     pcl_sensor_data::StopRecording::Response &res)
        // {
        //     this->pcl_sub.shutdown();
        //     ROS_INFO("Stopped recording");
        //     res.success = true;
        //     return true;
        // }


        //  void sub_to_laser(){

        //     std::string pointcloud_topic_name = this->pnh.param<std::string>("scanner_pointcloud_topic", "/arm/laser_scan");
        //     this->pcl_sub = nh.subscribe(pointcloud_topic_name, 10, &LaserProcessor::scanCallback, this);
            
        //  }
         bool start_recording(pcl_sensor_data::StartRecording::Request &req,
                             pcl_sensor_data::StartRecording::Request &res)
        {
            std::string pointcloud_topic_name = this->pnh.param<std::string>("scanner_pointcloud_topic", "/arm/laser_scan");
            this->pcl_sub = nh.subscribe(pointcloud_topic_name, 10, &LaserProcessor::scanCallback, this);
            this->sum_data.clear();
            ROS_INFO("Started recording");

            // res.success = true;
            return true;
        }


        bool stop_recording(pcl_sensor_data::StopRecording::Request &req,
                            pcl_sensor_data::StopRecording::Request &res)
        {
            this->pcl_sub.shutdown();
            ROS_INFO("Stopped recording");

            // res.success = true;
            return true;
        }

        // bool process_pointcloud(pcl_sensor_data::ProcessPointcloud::Request &req,
        //                         pcl_sensor_data::ProcessPointcloud::Response &res)
        // {
        //     this->cumulative_pointcloud.header = this->transformed.header;
        //     this->cumulative_pointcloud.header.stamp = ros::Time(0);
        //     this->cumulative_pointcloud.data = this->sum_data;
        //     this->cumulative_pointcloud.height = 1;
        //     this->cumulative_pointcloud.point_step = this->transformed.point_step;
        //     this->cumulative_pointcloud.width = this->sum_data.size()/cumulative_pointcloud.point_step;
        //     this->cumulative_pointcloud.is_dense = true;
        //     this->cumulative_pointcloud.row_step = this->sum_data.size();
        //     this->cumulative_pointcloud.fields = this->transformed.fields;
        //     ros::Rate loop_rate(10);
        //     this->result_pub = pnh.advertise<sensor_msgs::PointCloud2>("results", 1000);
        //     ROS_INFO("Preparing to publish result");
        //     // TODO: Remove this loop
        //     while (true)
        //     {
        //         this->cumulative_pointcloud.header.stamp = ros::Time(0);
        //         this->result_pub.publish(this->cumulative_pointcloud);
        //         loop_rate.sleep();
        //         ros::spinOnce();
        //     }

        //     res.success = true;
        //     return true;
        // }

         bool process_pointcloud(pcl_sensor_data::ProcessPointcloud::Request &req, pcl_sensor_data::ProcessPointcloud::Request &res)
        {
            this->cumulative_pointcloud.header = this->transformed.header;
            this->cumulative_pointcloud.header.stamp = ros::Time(0);
            this->cumulative_pointcloud.header.frame_id = this->frame_id;
            this->cumulative_pointcloud.points = this->sum_data;
            // this->cumulative_pointcloud.height = 1;
            // this->cumulative_pointcloud.point_step = this->transformed.point_step;

                // std::cout << this->sum_data.size() << std::endl;
            // this->cumulative_pointcloud.width = this->sum_data.size(); ///cumulative_pointcloud.point_step
         
            // this->cumulative_pointcloud.is_dense = true;
            // this->cumulative_pointcloud.row_step = this->sum_data.size();
            // this->cumulative_pointcloud.fields = this->transformed.fields;
            ros::Rate loop_rate(10);
            this->result_pub = pnh.advertise<sensor_msgs::PointCloud>("results", 1000);
            ROS_INFO("Preparing to publish result");
            // TODO: Remove this loop
            // res.success = true;
             std::ofstream file("segment1.csv");
           

            for (const auto& point : cumulative_pointcloud.points) {

                float x = point.x;
                float y = point.y;
                float z = point.z;
                // Do something with the x, y, and z values
                //...

                file << x << "," << y << "," << z << std::endl;
            }

            file.close();




            while (ros::ok())
            {
                this->cumulative_pointcloud.header.stamp = ros::Time(0);
                this->result_pub.publish(this->cumulative_pointcloud);
              

                loop_rate.sleep();
                ros::spinOnce();
            }

           
           return true;
        }

        // void second_callback(const sensor_msgs::PointCloudConstPtr &scan_in)
        // {

        //     sensor_msgs::PointCloud2 transformed;
        //     sensor_msgs::PointCloud2 res;
        //     ros::Time time = ros::Time(0);
        //     try{
        //         this->listener.waitForTransform("/world", this->frame_id, scan_in->header.stamp, ros::Duration(0.1));
        //         this->listener.lookupTransform("/world", this->frame_id,  scan_in->header.stamp, stamped_transform);
        //     } catch(tf::TransformException ex){
        //         ROS_ERROR("%s", ex.what());
        //         ROS_ERROR("RIP");
        //         return;
        //     }
        //     pcl_ros::transformPointCloud("world", *scan_in, this->transformed, this->listener);

        //     this->sum_data.reserve(this->sum_data.size() + this->transformed.data.size());
        //     this->sum_data.insert(this->sum_data.end(), this->transformed.data.begin(), this->transformed.data.end());
        // }



        // void pcl_callback(const sensor_msgs::LaserScan &scan_in)
        // {
            
        //     sensor_msgs::PointCloud2 transformed;
           
        //     ros::Time time = ros::Time(0);
        //     try{
        //         this->listener.waitForTransform("/world", this->frame_id, scan_in->header.stamp, ros::Duration(0.1));
        //         this->listener.lookupTransform("/world", this->frame_id,  scan_in->header.stamp, stamped_transform);
        //     } catch(tf::TransformException ex){
        //         ROS_ERROR("%s", ex.what());
        //         ROS_ERROR("RIP");
        //         return;
        //     }
        //     pcl_ros::transformPointCloud("world", *scan_in, this->transformed, this->listener);

        //     this->sum_data.reserve(this->sum_data.size() + this->transformed.data.size());
        //     this->sum_data.insert(this->sum_data.end(), this->transformed.data.begin(), this->transformed.data.end());
        // }

        

        // void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        // {
        //     sensor_msgs::PointCloud cloud;
        //     this->projector_.projectLaser(*scan_in, cloud);

        //     // Do something with cloud.
        //     // this->second_callback(cloud);
        //     result_pub.publish(cloud);

        // }

        


        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
        if(!listener_.waitForTransform(
                scan_in->header.frame_id,
                "/turntable_plate",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0))){
            return;
        }

        
        sensor_msgs::PointCloud cloud;
        projector_.transformLaserScanToPointCloud("/turntable_plate",*scan_in,
                cloud,listener_);
        // this->result_pub = pnh.advertise<sensor_msgs::PointCloud>("results", 1000);
    
        this->transformed = cloud;
        // this->result_pub.publish(cloud);
        
         this->sum_data.reserve(this->sum_data.size() + this->transformed.points.size());
         this->sum_data.insert(this->sum_data.end(), this->transformed.points.begin(), this->transformed.points.end());

   

        }
};






int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_processing_services");

    ros::NodeHandle nh;



	// ros::AsyncSpinner spinner(1);
	// spinner.start();

	LaserProcessor laser_processor;
    // laser_processor.start_recording(true, true);

   
    // // Set the loop duration.
    // ros::Duration(0.5).sleep();

    // // Get the current time.
    // float start_time = ros::Time::now().toSec();

    // float elapsedTime = 0;
    // // Loop until the elapsed time exceeds the loop duration.
    // while (ros::ok() && elapsedTime <= 20.0)
    // {
    //     // Do something here.

    //     // Process any incoming messages.
    //     // std::cout << elapsedTime << std::endl;
    //     elapsedTime = ros::Time::now().toSec() - start_time;
    //     ros::spinOnce();
    // }
  
    // laser_processor.stop_recording(true, true);
    // laser_processor.process_recording();

    // std::cout << "starting to show the cumulative" << std::endl;
  
    // laser_processor.stop_recording(true, true);

  
    // laser_processor.test_service_cloud(true, true);

    ros::spin();
	// ros::waitForShutdown();
	// spinner.stop();
	return 0;
}