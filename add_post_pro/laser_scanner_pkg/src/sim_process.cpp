
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

/*
    Topic name: /scancontrol2900
    Type: sensor_msgs/LaserScan
    header: 
        seq: 12
        stamp: 
            secs: 92
            nsecs: 603000000
        frame_id: "scancontrol2900-100_laser"
    angle_min: -0.39269900322
    angle_max: 0.39269900322
    angle_increment: 0.000767739955336
    time_increment: 0.0
    scan_time: 0.0
    range_min: 0.10000000149
    range_max: 0.5
    ranges[]
    intensities[]
*/
class LaserScannerSim
{
    private:
        ros::NodeHandle nh, pnh;		//pnh = parameter node handle
        std::string frame_id;
        unsigned int seq;

        ros::Subscriber scanner_frame;
        ros::Publisher scan_discrete;
        ros::ServiceServer grab_scan;

        sensor_msgs::LaserScanPtr current_scan;
        boost::shared_mutex update_frame_lock;

    public:
        LaserScannerSim() : nh("laser_scan_sim"), pnh("~"), seq(0)
        {
            ROS_INFO("[sim_scanner_services] Initializing...");

            std::string scan_topic_name = this->pnh.param<std::string>("laser_scanner_topic", "/scancontrol2900");
            this->scanner_frame = this->nh.subscribe(scan_topic_name, 20, &LaserScannerSim::scan_callback, this);

            std::string ls_srv_name = this->pnh.param<std::string>("laser_scan_service_name", "grab_scan");
            this->current_scan = this->nh.advertiseService(ls_srv_name, &LaserScannerSim::return_scan, this);

            ROS_INFO("[sim_scanner_services] Initializing...");
        }

    private:

        void scan_callback(const sensor_msgs::LaserScanConstPtr& frame)
        {
            boost::upgrade_lock<boost::shared_mutex> _lock(this->update_frame_lock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(_lock);
			this->current_scan = sensor_msgs::LaserScanPtr(new sensor_msgs::LaserScan(*frame));
        }

        void return_scan(laser_scanner_pkg::FetchOneScan::Request &req,
                          laser_scanner_pkg::FetchOneScan::Response &res)
        {
            ROS_INFO("[sim_scanner_services] Sending scan");
            res.success = true;
            res.reason = "Success!";
            res.depth = *this->current_scan;
            return true;
        }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_process");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    LaserScannerSim scanner_sim;

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}