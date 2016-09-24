// NOTE: memory leak in readComPort : mallocs a buffer but never frees it

#include "ros/ros.h"
//#include <imu/Orientation.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include "com_port.h"
#include <unistd.h>
#include <vector>
#include <stdlib.h>

const char WILD_CHAR = '%';
int DEBUG = 1;

using namespace std;

double toRadian(double degrees);

// simple wrapper for a ComPort so that you can easily read character by character and "peek" at the next character
class ComStream {
	private:
		char peekChar;
		ComPortHandle handle;
	public:
		ComStream(ComPortHandle h) : handle(h), peekChar(-1) {}
		char peek() { // returns the next character without taking it off the stream
			while (peekChar == -1) peekChar = readByte(handle);
			return peekChar;
		}
		char next() { // returns the next character (and removes it from the stream); blocks until a character is available
			char value = peek();
			peekChar = -1;
			return value;
		}			
				
};

// returns true if the input char is in [0-9] or '-' or '.' (i.e. could it be part of a number?)
bool isNumeric(char c) {
	if (c >= '0' && c <= '9') return true;
	if (c == '-' || c == '.') return true;
	return false;
}

// Basically a poor man's regexp
// reads from the port looking for a sequence that matches the pattern string.
// WILD_CHAR is used in the pattern as a placeholder for any double value; the doubles captured will be read into the values array.
// WILD_CHAR can be 'escaped' in the pattern by typing it twice.
// NOTE: readDoubles has no memory; if it is halfway through matching a pattern and finds a mismatch, it will NOT go back to try to match against any of the previously-read characters
// eg. pattern "abac" will NOT match against stream data "ababac", since "aba" is discarded as a potential match, and matching starts again with "bac"
int readDoubles(ComStream port, string pattern, double* values) {
	int i = 0, v = 0, len = pattern.length();
	while (i < len) { // i is an index to keep track of where we are in the pattern
		if (pattern[i] == WILD_CHAR) { // we reached a WILD_CHAR in the pattern
			i++;
			if (i == len || pattern[i] != WILD_CHAR) { // the WILD_CHAR is not escaped
				string s;
				while (isNumeric(port.peek())) { // keep reading characters that look like they are part of a number
					s += port.next();
				}
				values[v++] = atof(s.c_str()); // convert the read characters into a double and store it in 'values'
				continue;
			}
		}
	
		if (pattern[i] == port.next()) { // if no unescaped WILD_CHAR, ensure that the next stream character matches the pattern
			i++;
                } else {
			// Exit, this wasn't the right command
			return 0;
		}
	}
	return v; // if we make it out of the loop, we found a match! return the number of values captured
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "imu");
	ros::NodeHandle n;
        ros::NodeHandle privateHandle("~");

        string portName;
        privateHandle.param<string>("port", portName, "/dev/ttyUSB1");

	//ros::Publisher orientationPublisher = n.advertise <imu::Orientation> ("art/imu", 1000);
	ros::Publisher yawPublisher = n.advertise <std_msgs::Float64> ("imu_data", 1000); // A more convenient format

	// Load parameters
	n.param<int>("/art/debug", DEBUG, 0);

	// connect to imu	
	ros::Rate imuCheckRate(0.5);	
	ComPortHandle port = -1;
	while (port == -1 && ros::ok()) {
            port = OpenComPort(portName.c_str(), false);
		if (port == -1) {
			ROS_FATAL("[imu] Unable to connect to imu!");
		} else {
			ROS_INFO("[imu] Connected to imu!");
		}
		imuCheckRate.sleep();
	}

	double values[5];
	ComStream strm(port);
	//imu::Orientation fullMsg;
        std_msgs::Float64 yawMsg;

	double yawOffset = 0;
	while (ros::ok()) { // continually read values from IMU and publish them

		int n = readDoubles(strm, "Yaw: % Pitch: % Roll: %", values);

		if (n == 3) {
			// Increase by 180 degrees to remove negative values
			values[0] += 180;

			if (DEBUG) {
                            ROS_INFO("(Yaw, Pitch, Roll) = (%f, %f, %f)", values[0], values[1], values[2]);
                        }

			//fullMsg.yaw = values[0];
			//fullMsg.pitch = values[1];
			//fullMsg.roll = values[2];	
                        cout << "(YAW, PITCH, ROLL) = " << values[0] << ", " << values[1] << ", " << values[2] << endl;
			//orientationPublisher.publish(fullMsg);

                        yawMsg.data = toRadian(values[0]);
                        yawPublisher.publish(yawMsg);
		}

		ros::spinOnce();
	}
	
	return 0;
}

double toRadian(double degrees) {
	return degrees * (M_PI / 180);
}
