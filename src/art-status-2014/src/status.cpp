#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <tf/transform_broadcaster.h>

#include <fstream>
#include <iostream>

void publishRobotGeometry(ros::Publisher robotGeometryPublisher, double robotWidth, double x, double y, double th);
//void publishMapToScanTransform(tf::TransformBroadcaster broadcaster, double x, double y, double th);
void processTrajectory(ros::Time lastTime, double vl, double vr);

// Store robot parameters
double robotWidth;

int main(int argc, char **argv) {	
	ros::init(argc, argv, "status");
	ros::NodeHandle n;
	
	// Load parameters	
	n.param<double>("/art/setup/robot_width", robotWidth, 0.2);
 
	// Prepare publishers
	ros::Publisher globalMapPublisher = n.advertise <nav_msgs::OccupancyGrid> ("art/debug/globalmap", 1000, true);
	ros::Publisher scanPublisher = n.advertise <sensor_msgs::LaserScan> ("scan", 1000);
	ros::Publisher robotGeometryPublisher = n.advertise <geometry_msgs::PolygonStamped> ("art/debug/robot_geometry", 1000);

	// Prepare tf
	//tf::TransformBroadcaster mapToScanTransformer;

	// Prepare the world map
	nav_msgs::OccupancyGrid map;
	map.header.frame_id = "/map";
	
	map.header.stamp = ros::Time::now();
	globalMapPublisher.publish(map);

	ros::Rate rate(50);

	while (ros::ok()) {
				
		publishRobotGeometry(robotGeometryPublisher, robotWidth, 0, 0, 0);

		//publishMapToScanTransform(mapToScanTransformer, x, y, th);
		ros::spinOnce();
		rate.sleep();
	}

}

// void publishMapToScanTransform(tf::TransformBroadcaster broadcaster, double x, double y, double th) {
// 	tf::Transform laserTransform;
// 	laserTransform.setOrigin(tf::Vector3(0, 0, 0));
// 	laserTransform.setRotation(tf::Quaternion(0, 0, 0));

// 	tf::Transform actualPositionTransform;
// 	actualPositionTransform.setOrigin( tf::Vector3(x, y, 0.0) );
// 	actualPositionTransform.setRotation( tf::Quaternion(0, 0, th) );
	
// 	broadcaster.sendTransform(tf::StampedTransform(laserTransform, ros::Time::now(), "map", "laser"));
// 	broadcaster.sendTransform(tf::StampedTransform(actualPositionTransform, ros::Time::now(), "map", "actual-position"));
// }

void publishRobotGeometry(ros::Publisher robotGeometryPublisher, double robotWidth, double x, double y, double th) {
	geometry_msgs::PolygonStamped robotGeometryMsg;

	robotGeometryMsg.header.stamp = ros::Time::now();
	robotGeometryMsg.header.frame_id = "/actual-position";

	geometry_msgs::Point32 pt;


	pt.x = 0 - (robotWidth / 2);
	pt.y = 0 + (robotWidth / 2);
	robotGeometryMsg.polygon.points.push_back(pt);

	pt.x = 0 + (robotWidth / 2);
	pt.y = 0 + (robotWidth / 2);
	robotGeometryMsg.polygon.points.push_back(pt);

	pt.x = 0 + (robotWidth / 2);
	pt.y = 0 - (robotWidth / 2);
	robotGeometryMsg.polygon.points.push_back(pt);

	pt.x = 0 - (robotWidth / 2);
	pt.y = 0 - (robotWidth / 2);
	robotGeometryMsg.polygon.points.push_back(pt);

	robotGeometryPublisher.publish(robotGeometryMsg);

}
