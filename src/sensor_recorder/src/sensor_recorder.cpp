#include <fstream>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

using namespace std;

double globalLatitude = 0;
double globalLongitude = 0;
double globalBearing = 0;

const char* filename = "sensorRecording.txt";
ofstream out;

// Callback for when we receive data from the GPS
void gpsCallback(const geometry_msgs::Point::ConstPtr& gpsLatLon)
{
    // Just set the global "current gps location"
    globalLatitude = gpsLatLon->x;
    globalLongitude = gpsLatLon->y;
}

// Callback for when we receive data from the IMU
void imuCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // Just set the global "current imu bearing"
    globalBearing = msg->data;
}

// Callback for when we receive a frame-synchronizer message
void readFrameCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int frameNumber = msg->data;

    out << "Frame: " << frameNumber
        << "; Lat: " << globalLatitude
        << "; Lon: " << globalLongitude
        << "; Bearing: " << globalBearing
        << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_recorder");
    ros::NodeHandle n;

    out.open(filename);

    //Subscribe to GPS data
    // We only need the most recent one, right? So queue size of 1 is okay?
    ros::Subscriber gpsSub = n.subscribe("gps_data", 1, gpsCallback);

    //Subscribe to IMU data
    // We only need the most recent one, right? So queue size of 1 is okay?
    ros::Subscriber imuSub = n.subscribe("imu_data", 1, imuCallback);

    //Subscribe to frame synchronizer messages
    ros::Subscriber readFrameSub = n.subscribe("read_frame", 100, readFrameCallback);

    ros::spin();
	
    return 0;
}

