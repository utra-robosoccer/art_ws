#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
//#include <ucontroller/ReceivedMsg.h>
//#include <imu/Orientation.h>
#include "std_msgs/Float64.h"

#include <tf/transform_broadcaster.h>

#include <fstream>
#include <iostream>

#define LIDAR_ANGLE_MIN -2.35619449615
#define LIDAR_ANGLE_MAX 2.35619449615
#define LIDAR_ANGLE_INCREMENT 0.00872664619237
#define LIDAR_TIME_INCREMENT 3.6968576751e-05
#define LIDAR_SCAN_TIME 0.019999999553
#define LIDAR_RANGE_MIN 0.00999999977648
#define LIDAR_RANGE_MAX 20.0

void loadGlobalMap(nav_msgs::OccupancyGrid *map, double *originX, double *originY);
void simulateHorizontalLidar(ros::Publisher lidarPublisher,  ros::Publisher scanPublisher, nav_msgs::OccupancyGrid *map, double x, double y, double th); 
void simulateIMU(ros::Publisher imuPublisher, double th);
bool isBlocked(const nav_msgs::OccupancyGrid *map, int x, int y);
bool robotInBlock(nav_msgs::OccupancyGrid *map, int centerX, int centerY);
void publishRobotGeometry(ros::Publisher robotGeometryPublisher, double robotWidth, double x, double y, double th);
void publishMapToScanTransform(tf::TransformBroadcaster broadcaster, double x, double y, double th);
void simulateGPS(ros::Publisher gpsPublisher, double x, double y, nav_msgs::OccupancyGrid *map);
//void uPublishCallback (const ucontroller::ReceivedMsg::ConstPtr& msg);
void uPublishCallback (const geometry_msgs::Point32::ConstPtr& msg);
void processTrajectory(ros::Time lastTime, double vl, double vr);
void resetPublishCallback(const std_msgs::Empty::ConstPtr& msg);
void runPublishCallback(const std_msgs::Empty::ConstPtr& msg);

// Status flags
bool crashed = false;
bool running = false;

// Store world coordinates
double x;
double y;
double th;

double initialX, initialY;

ros::Time stamp;
bool timeInitialized = false;

std::string dataDirectory;
std::string globalMapFileName;

struct Motor {
	double constant;

	int leftSpeedCommand;
	int rightSpeedCommand;
} motor;

// Store robot parameters
double robotWidth;

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "world");
	ros::NodeHandle n;

	// Load parameters	
	n.param<double>("/art/setup/robot_width", robotWidth, 0.2);
	n.param<double>("/art/setup/motor_constant", motor.constant, 0);
	n.param<std::string>("/art/simulation/data_dir", dataDirectory, "./data/");
	n.param<std::string>("/art/simulation/global_map_file_name", globalMapFileName, "globalMap.txt");
 
	// Prepare publishers
	ros::Publisher globalMapPublisher = n.advertise <nav_msgs::OccupancyGrid> ("art/debug/globalmap", 1000, true);
	ros::Publisher horizontalLidar = n.advertise <sensor_msgs::LaserScan> ("art/lidar/horizontal", 1000);
	ros::Publisher scanPublisher = n.advertise <sensor_msgs::LaserScan> ("scan", 1000);
	ros::Publisher robotGeometryPublisher = n.advertise <geometry_msgs::PolygonStamped> ("art/debug/robot_geometry", 1000);
	ros::Publisher imuPublisher = n.advertise <std_msgs::Float64>("imu_data", 1000);
	ros::Publisher gpsPublisher = n.advertise <geometry_msgs::Point32>("gps_data", 1000);

	// Prepare listeners
	ros::Subscriber uListener = n.subscribe ("simulator_cmds", 1000, uPublishCallback);
	//ros::Subscriber uListener = n.subscribe ("art/ucontroller/publish", 1000, uPublishCallback);
	ros::Subscriber resetListener = n.subscribe ("art/simulation/reset", 1000, resetPublishCallback);
	ros::Subscriber runListener = n.subscribe ("art/simulation/run", 1000, runPublishCallback);

	// Prepare tf
	tf::TransformBroadcaster mapToScanTransformer;

	std::cout << globalMapFileName << " " << dataDirectory << std::endl;
	// Prepare the world map
	nav_msgs::OccupancyGrid map;
	map.header.frame_id = "/map";
	// Load the global map into memory
	loadGlobalMap(&map, &x, &y);

	initialX = x;
	initialY = y;
	
	map.header.stamp = ros::Time::now();
	globalMapPublisher.publish(map);

	// Set the robot starting position at the map origin
	th = 0;//M_PI;//M_PI / 4;

	//ROS_INFO("Running LIDAR...");
	ros::Rate rate(10);
	
	int counter = 0;

	while (ros::ok()) {

		if (running) ROS_INFO("ITERATION");
		else if (crashed) ROS_ERROR("Rover crashed! Simulation stopped.");
		else ROS_INFO("Simulation stopped ...");

		//ROS_INFO("Running: simulateHorizontalLidar");
		simulateHorizontalLidar(horizontalLidar, scanPublisher, &map, x, y, th);	
		//ROS_INFO("Running: simulateIMU");
		simulateIMU(imuPublisher, th);
		simulateGPS(gpsPublisher, x, y, &map);
				
		//ROS_INFO("Running: publishRobotGeometry");
		publishRobotGeometry(robotGeometryPublisher, robotWidth, x, y, th);

		//ROS_INFO("Running: publishMapToScanTransform");
		publishMapToScanTransform(mapToScanTransformer, x, y, th);
		
		//ROS_INFO("ROBOT POSE: X: %f, Y: %f", x, y);	
		// Process the trajectory
		//ROS_INFO("Running: processTrajectory");
		
		if (running) {
		  processTrajectory(ros::Time::now(), motor.leftSpeedCommand, motor.rightSpeedCommand);
		}
		if (isBlocked(&map, x / map.info.resolution, y / map.info.resolution)) {
		  crashed = true;
		  running = false;
		}

		counter++;
		//std::cout << "World: " << counter << std::endl;

		//ROS_INFO("Running: ros::spinOnce");
		ros::spinOnce();
		//ROS_INFO("Running: rate.sleep()");
		rate.sleep();
	}

}

void resetPublishCallback(const std_msgs::Empty::ConstPtr& msg) {
	x = initialX;
	y = initialY;
	th = 0;
	crashed = false;
	running = false;
	timeInitialized = false;
}

void runPublishCallback(const std_msgs::Empty::ConstPtr& msg) {
  if (!crashed) running = true;
  timeInitialized = false;
}

void simulateIMU(ros::Publisher imuPublisher, double th) {
	//imu::Orientation orientationMsg;
	std_msgs::Float64 yawMsg;

	yawMsg.data = -th;

	imuPublisher.publish(yawMsg);
}

// Compare with gpsDelta in path_planner.cpp
// This is the "inverse" in some sense
// But it assumes that the initial location of the rover (initalX, initialY)
// gets mapped to 0 latitude and 0 longitude
void simulateGPS(ros::Publisher gpsPublisher, double x, double y, nav_msgs::OccupancyGrid *map) {
  const double radiusOfEarth = 6371000; // metres
  
  // TODO: FIXME
  double scale = 1 / 0.05;
  double deltaX = (x - initialX) * scale;
  double deltaY = (y - initialY) * scale;
  double latDelta = deltaY / radiusOfEarth;
  double averageLat = (latDelta + 0.0) / 2.0;
  double lonDelta = deltaX / radiusOfEarth / cos(averageLat);

  geometry_msgs::Point32 p;
  p.x = latDelta;
  p.y = lonDelta;
  gpsPublisher.publish(p);
}

/*
void uPublishCallback (const ucontroller::ReceivedMsg::ConstPtr& msg) {
	if (msg->msgType == (unsigned int)'s') {
		motor.leftSpeedCommand = (int)msg->payload[0];
		motor.rightSpeedCommand = (int)msg->payload[1];
	}
}*/

void uPublishCallback (const geometry_msgs::Point32::ConstPtr& msg) {
	motor.leftSpeedCommand = msg->x;
	motor.rightSpeedCommand = msg->y;
}

void processTrajectory(ros::Time lastTime, double vl, double vr) {
  //ROS_INFO("PROCESS_TRAJECTORY: %f %f", vl, vr);
	if (timeInitialized) {
	// Convert motor command to velocity
		vl /= motor.constant;
		vr /= motor.constant;
	
		// Y (forward) is the average of the motor speeds
		double dy = (vl + vr) / 2;
		// Turning left is positive
		// Use the half-width, process from the center of the robot
		double dth = (vr - vl) / (robotWidth / 2);

		ros::Duration dt = lastTime - stamp;
	
		for (int i = 0; i < 1; i++) {
			y += dy * dt.toSec()  * sin(th);
			x += dy * dt.toSec() * cos(th);
	
			th += dth * dt.toSec();
		}
	}
	timeInitialized = true;
	stamp = lastTime;
}

void publishMapToScanTransform(tf::TransformBroadcaster broadcaster, double x, double y, double th) {
	tf::Transform laserTransform;
	laserTransform.setOrigin(tf::Vector3(0, 0, 0));
	laserTransform.setRotation(tf::Quaternion(0, 0, 0));

	tf::Transform actualPositionTransform;
	actualPositionTransform.setOrigin( tf::Vector3(x, y, 0.0) );
	actualPositionTransform.setRotation( tf::Quaternion(0, 0, th) );
	
	broadcaster.sendTransform(tf::StampedTransform(laserTransform, ros::Time::now(), "map", "laser"));
	broadcaster.sendTransform(tf::StampedTransform(actualPositionTransform, ros::Time::now(), "map", "actual-position"));
}

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

void simulateHorizontalLidar(ros::Publisher lidarPublisher, ros::Publisher scanPublisher, nav_msgs::OccupancyGrid *map, double x, double y, double th) {
	sensor_msgs::LaserScan scanMsg;

	scanMsg.header.frame_id = "/laser";
	scanMsg.header.stamp = ros::Time::now();
	
	scanMsg.angle_min = LIDAR_ANGLE_MIN;
	scanMsg.angle_max = LIDAR_ANGLE_MAX;
	scanMsg.angle_increment = LIDAR_ANGLE_INCREMENT;
	
	scanMsg.time_increment = LIDAR_TIME_INCREMENT;
	
	scanMsg.scan_time = LIDAR_SCAN_TIME;

	scanMsg.range_min = LIDAR_RANGE_MIN;
	scanMsg.range_max = LIDAR_RANGE_MAX;

	double testAngle = th;
	int rangeIndex = 0;
	for (double lidarAngle = LIDAR_ANGLE_MIN; lidarAngle < LIDAR_ANGLE_MAX; lidarAngle += LIDAR_ANGLE_INCREMENT) {
	
		// The angle in the world frame	
		testAngle = th + lidarAngle;

		//ROS_INFO("Examining angle %f\n", testAngle);

		double k = std::sin(testAngle);
		double c = std::cos(testAngle);

		double dx = 0;
		double dy = 0;
		bool pushed = false;
		for (double ds = 0; ds < LIDAR_RANGE_MAX; ds += 0.1) {
			// Get the x and y coordinates of the trace, then add the starting position
			dy = k * ds + y;
			dx = c * ds + x;

			// Place dx and dy onto a tile
			dy = floor(dy / map->info.resolution);
			dx = floor(dx / map->info.resolution);

			//ROS_INFO("Exploring cell: %f %f\n", dx, dy);

			
			if (dy >= 0 && dx >= 0 && dy < map->info.height && dx < map->info.width) {
				//ROS_INFO("%f, %f", dy, dx);
				if (map->data[dy * map->info.width + dx] == 100) {
					scanMsg.ranges.push_back(ds);
					pushed = true;
					//ROS_INFO("Found a block at %f\n", ds); 
					break;
				}
			}
		}

		if (pushed == false) {
		        //ROS_INFO("Could not find anything at this angle!\n");
			scanMsg.ranges.push_back(42);
		}
		// If nothing was found in the trace, it will just use the default value in the range (0)
		rangeIndex++;
		// Get a value for the dx and dy after a certain distance, since line is straight the whole way, we can just add dx and dy to x and y without more calculations
		
	}
	scanMsg.ranges[0] = 5;
	scanMsg.ranges[scanMsg.ranges.size() - 1] = 5;
	scanPublisher.publish(scanMsg);
	scanMsg.header.frame_id = "/actual-position";
	lidarPublisher.publish(scanMsg);
}

void loadGlobalMap(nav_msgs::OccupancyGrid *map, double *originX, double *originY) {
	std::ifstream file((dataDirectory + globalMapFileName).c_str(), std::ios::in);

	if (!file) {
		ROS_FATAL("Could not read global map from file (%s)!", (dataDirectory + globalMapFileName).c_str());
		exit(1);
	}
	
	// Read 
	int width;
	int height;
	
	file >> width;
	file >> height;

	map->info.width = width;
	map->info.height = height;
	map->info.resolution = 0.5;
	map->data.resize(map->info.width * map->info.height);
	
	int x = 0;
	int y = 0;
	int tile = 0;
	// Skip over the \n on the first line
	char letter = (char)file.get();
	
	while ((int)(letter = (char)file.get()) != -1) {
		// Note: The y-coordinate is inverted so that the map is built with (0, 0) in the lower-left corner
		tile = (map->info.height - y - 1) * map->info.width + x;
		x++;	
		//file.get(letter);
		if (letter == 'X') {
			map->data[tile] = 100;
		} else if (letter == '.') {
			map->data[tile] = 0;
		} else if (letter == 'O') {
			*originX = (x - 1) * map->info.resolution;
			*originY = (map->info.height - y - 1) * map->info.resolution;
			map->info.origin.position.x = 0;//x - 1;
			map->info.origin.position.y = 0;//map->info.height - y - 1;
		} else if (letter == '\n') {
			x = 0;
			y++;
		}

	}

	file.close();
	ROS_INFO("World map loaded.");
}

bool robotInBlock(nav_msgs::OccupancyGrid *map, int centerX, int centerY) {
	int robotHalfWidth = (double)((robotWidth / 2) / map->info.resolution) + 1;
	//ROS_INFO("Half width: %d", robotHalfWidth);
	for (int x = centerX - robotHalfWidth; x < centerX + robotHalfWidth; x++) {
		for (int y = centerY - robotHalfWidth; y < centerY + robotHalfWidth; y++) {
			if (isBlocked(map, x, y)) {
				return true;
			}
		}
	}
	return false;
}

bool isBlocked(const nav_msgs::OccupancyGrid *map, int x, int y) {
	int tile = y * map->info.width + x;
	if (tile >= map->data.size()) {
		return true;
	}
	if (map->data[y * map->info.width + x] == 100) {
		return true;
	}
	return false;
}

