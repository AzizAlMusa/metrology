#include <ros/ros.h>
#include <pcl_sensor_data/angleAdjust.h>
#include <pcl_sensor_data/avgZDistance.h>
#include <pcl_sensor_data/laserMax.h>


int main (int argc, char** argv)
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;

	ros::ServiceClient client_ang = n.serviceClient<pcl_sensor_data::angleAdjust> ("return_angle");
	ros::ServiceClient client_dist = n.serviceClient<pcl_sensor_data::avgZDistance> ("return_distance");
	ros::ServiceClient client_laser = n.serviceClient<pcl_sensor_data::laserMax> ("return_laser");

	pcl_sensor_data::angleAdjust srv_angle;
	pcl_sensor_data::avgZDistance srv_dist;
	pcl_sensor_data::laserMax srv_laser;

	srv_angle.request.req = true;
	srv_dist.request.req = true;
	srv_laser.request.req = true;

	if(client_ang.call(srv_angle)) {
		ROS_INFO("Angle: %e", srv_angle.response.angle);
		sleep(5.0);
	} else {
		ROS_INFO("Error");
	}

	if(client_dist.call(srv_dist)) {
		ROS_INFO("Distance: %e", srv_dist.response.distance);
		sleep(5.0);
	} else {
		ROS_INFO("Error");
	}

	if(client_laser.call(srv_laser)) {
		ROS_INFO("Laser Max: {X: %e, Z: %e}", srv_laser.response.laserMax[0], srv_laser.response.laserMax[1]);
		sleep(5.0);
	} else {
		ROS_INFO("Error");
	}
}