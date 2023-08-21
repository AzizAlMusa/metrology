#include <array>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

// Msg files
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Srv files
#include <pcl_sensor_data/angleAdjust.h>
#include <pcl_sensor_data/avgZDistance.h>
#include <pcl_sensor_data/laserMax.h>

// Global variables
ros::Publisher pub;
tf::TransformListener *tf_listener;
std::array<std::vector<double>, 2> xZArray;
double angAdjust;
double avgZDist;
std::vector<double> laserMax;
// TODO: Use class to encapsulate the variables

// Point Cloud Pre-processing -------------------------------------------------

// returns a vector of len 2 arrays with spurious discontinuous values removed
std::array<std::vector<double>, 2> filteredCloud(
	const std::array<std::vector<double>, 2> &xZArray)
{
	std::array<std::vector<double>, 2> filteredArray;

	for (int i=xZArray[1].size()-1; i>0; i--)
	{
		//std::cout << xZArray[1].size() <<std::endl;
		//std::cout << xZArray[1][i-1] << std::endl;
		//std::cout << xZArray[1][i] << std::endl;

		// make sure there is no jump in values between adjacent points
		if ((abs(xZArray[1][i-1] - xZArray[1][i])) < 10)
		{
			filteredArray[0].push_back(xZArray[0][i]);
			filteredArray[1].push_back(xZArray[1][i]);
		}
		else
		{
			break;
		}
	}

		/*for (int j = 0; j < 2;j++)
		{
			for (auto i : filteredArray[j])
			{
				std::cout << i << std::endl;
			}
			std::cout << std::endl;
		}
		*/

		return filteredArray;
}

// returns a vector of len 2 arrays with spurious zero values removed
std::vector<std::array<double, 2>> refinedCloud(
	const std::array<std::vector<double>, 2> &xZArray)
{
	std::vector<std::array<double, 2>> zDeviation;
	std::array<double, 2> zDist;

	for (int i=0; i<xZArray[1].size(); i++)
	{
		zDist[0] = xZArray[0][i];
		zDist[1] = xZArray[1][i];

		// remove spurious zero values
		if (abs(zDist[1]) > 1e-6);
		{
			zDeviation.push_back(zDist); //? WHATS THIS?
		}
	}
	return zDeviation;
}

// Inference Calculation from Point Clouds ------------------------------------

// Assuming the sensor is orthogonal to a flat surface
// Calculate the the average distance between the surface & the sensor
double averageZDist(const std::array<std::vector<double>, 2> &xZArray)
{
	double zDist;
	double zSum = 0;
	double zCounter = 0;
	double avgDist;

	for (int i = 0; i<xZArray[1].size(); i++)
	{
		zDist = xZArray[1][i];
		if (zDist != 0)
		{
			zSum += zDist;
			zCounter += 1;
		}
	}
	avgDist = zSum / zCounter;

	return avgDist;
}

// Assuming the sensor scanning a flat surface
// Calculate the incline of the surface wrt the sensor
double averageSlope(const std::array<std::vector<double>, 2> &xZArray)
{
	std::vector<std::array<double, 2>> zDist;
	int stepSize = 5;
	double slope;
	double slopeSum = 0;
	int slopeCounter = 0;
	double avgSlope;

	zDist = refinedCloud(xZArray);
	std::cout << "zDist Size: " << zDist.size() << std::endl;

	for (int i=stepSize; i<zDist.size(); i++)
	{
		slope = (zDist[i][1] - zDist[i-stepSize][1]) /
			(zDist[i][0] - zDist[i-stepSize][0]);

		slopeSum += slope;
		slopeCounter += 1;
	}
	avgSlope = slopeSum / slopeCounter;

	std::cout << "slopeCounter = " << slopeCounter << std::endl;
	std::cout << "slopeSum = " << slopeSum << std::endl;
	std::cout << "Average Slope = " << avgSlope << std::endl;

	return avgSlope;
}

// Find the adjustment angle to make the sensor Orthogonal to
// the scanned surface
double angleAdjust(const std::array<std::vector<double>, 2> &xZArray)
{
	double avgSlope = averageSlope(xZArray);
	double angle = -atan(avgSlope);		// move in opposite direction

	return angle;
}

// Assuming the scanning beam hit two flat surfaces that are connected
// The point with the max Z-distance is the connection point
std::vector<double> getLaserMax(const std::array<std::vector<double>, 2> &xZArray)
{
	double distanceMax;
	int indexMax;
	std::vector<double> laserMax;

	distanceMax = *std::max_element(xZArray[1].begin(), xZArray[1].end());
	indexMax = std::distance(xZArray[1].begin(),
		std::max_element(xZArray[1].begin(), xZArray[1].end()));
	laserMax = { xZArray[0][indexMax], distanceMax };

	//std::cout << "Maximum Index = " << indexMax << "Max X-Value = "
	//	<< laserMax[0] << "Max Distance = " << laserMax[1] << std::endl;

	return laserMax;
}

// ROS Subscriber Callbacks ---------------------------------------------------

// Subscribe to "/arm/laser_scan" laser_scan msg
void laserScan_cb (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	// Project 2D Laser Scan into 3D Pointclouds
	laser_geometry::LaserProjection projector_;
	sensor_msgs::PointCloud2::Ptr inMsg(new sensor_msgs::PointCloud2());
	projector_.projectLaser(*scan_in, *inMsg);

	// Use 'Weldingsensor' frame
	sensor_msgs::PointCloud2::Ptr outMsg(new sensor_msgs::PointCloud2());
	pcl_ros::transformPointCloud("scancontrol2900-50_laser", *inMsg, *outMsg, *tf_listener);

	// iterators
	sensor_msgs::PointCloud2Iterator<float> out_x(*outMsg, "x");
	sensor_msgs::PointCloud2Iterator<float> out_y(*outMsg, "y");
	sensor_msgs::PointCloud2Iterator<float> out_z(*outMsg, "z");

	// output file
	std::ofstream file;
	file.open("arrayData.csv");
	file << "STREAM START\n";

	xZArray[0].clear();
	xZArray[1].clear();
	for (size_t i=0; i<(inMsg->height * inMsg->width);
		++i, ++out_x, ++out_y, ++out_z)
	{
		// std::cout << "out_x: " << *out_x << std::endl;
		// std::cout << "out_y: " << *out_y << std::endl;
		// std::cout << "out_z: " << *out_z << std::endl;

		xZArray[0].push_back(*out_x);
		xZArray[1].push_back(*out_z);

		file << *out_x << "," << *out_z << "\n";
	}
	file.close();

	pub.publish(outMsg);

	angAdjust = angleAdjust(xZArray);
	avgZDist = averageZDist(xZArray);
	laserMax = getLaserMax(xZArray);

	std::cout << "------------ ANGLE: " << angAdjust << std::endl;
	std::cout << "------------ AVG DISTANCE: " << avgZDist << std::endl;
	std::cout << "------------ MAX X DIST: " << laserMax[0] << std::endl;
	std::cout << "------------ MAX Z DIST: " << laserMax[1] << std::endl;
}

// ROS Service Callbacks ------------------------------------------------------

bool returnAngle(pcl_sensor_data::angleAdjust::Request &req,
				 pcl_sensor_data::angleAdjust::Response &res)
{
	res.angle = angAdjust;
	ROS_INFO("sending back response: [%e]", res.angle);

	return true;
}

bool returnDistance(pcl_sensor_data::avgZDistance::Request &req,
					pcl_sensor_data::avgZDistance::Response &res)
{
	res.distance = avgZDist;
	ROS_INFO("sending back response: [%e]", res.distance);

	return true;
}

bool returnLaser(pcl_sensor_data::laserMax::Request &req,
				 pcl_sensor_data::laserMax::Response &res)
{
	res.laserMax = laserMax;
	ROS_INFO("sending back response: X: %e, Z: %e", laserMax[0], laserMax[1]);

	return true;
}

// Main Loop ------------------------------------------------------------------
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "access_data");
  ros::NodeHandle nh;

  // tf listener
  tf_listener = new tf::TransformListener();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/arm/laser_scan", 1, laserScan_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_pointcloud", 1);

  // ROS servers that returns slope angle, average distance, & max distance
  ros::ServiceServer srv_angle = nh.advertiseService("return_angle",
  	returnAngle);
  ros::ServiceServer srv_distance = nh.advertiseService("return_distance",
  	returnDistance);
  ros::ServiceServer srv_laser = nh.advertiseService("return_laser",
  	returnLaser);

  // Spin
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  delete tf_listener;
}