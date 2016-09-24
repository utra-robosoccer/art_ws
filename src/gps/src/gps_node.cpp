// NOTE: memory leak in readComPort : mallocs a buffer but never frees it

#include <iostream>
#include "com_port.h"
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"

const double PI = 3.14159265359;

using namespace std;

struct GpsInfo
{
    int latDegrees;
    double latArcMinutes;
    int lonDegrees;
    double lonArcMinutes;
    bool north;
    bool east;
};

void stringSplit(const string& str, char delim, vector<string>& parts)
{
    stringstream strm(str);
    string item;
    while (getline(strm, item, delim)) {
        parts.push_back(item);
    }
}

bool parseDouble(const string& str, double& output)
{
    stringstream strm(str);
    strm >> output;
    return !strm.fail() && strm.eof();
}

bool parseInt(const string& str, int& output)
{
    stringstream strm(str);
    strm >> output;
    return !strm.fail() && strm.eof();
}

bool parseLon(const string& str, int& lonDegrees, double& lonArcMinutes)
{
    if (!parseInt(str.substr(0, 3), lonDegrees)) return false;
    if (!parseDouble(str.substr(3), lonArcMinutes)) return false;
    return true;
}

bool parseLat(const string& str, int& latDegrees, double& latArcMinutes)
{
    if (!parseInt(str.substr(0, 2), latDegrees)) return false;
    if (!parseDouble(str.substr(2), latArcMinutes)) return false;
    return true;
}

bool parseDir(const string& str, char trueChar, char falseChar, bool& output)
{
    if (str.length() != 1) return false;
    if (str[0] != trueChar && str[0] != falseChar) return false;
    output = (str[0] == trueChar);
    return true;
}

bool getLatLongFromLine(const string& line, GpsInfo& gps)
{
    if (line.length() < 6) return false;
    if (line.substr(0, 6) != "$GPGGA") return false;

    vector<string> parts;
    stringSplit(line, ',', parts);

    if (parts.size() < 6) return false;
    if (!parseLat(parts[2], gps.latDegrees, gps.latArcMinutes)) return false;
    if (!parseDir(parts[3], 'N', 'S', gps.north)) return false;
    if (!parseLon(parts[4], gps.lonDegrees, gps.lonArcMinutes)) return false;
    if (!parseDir(parts[5], 'E', 'W', gps.east)) return false;

    return true;
}

void printMatch(bool match, const string& line, const GpsInfo& gps)
{
    cout << (match ? "MATCH: " : "NO MATCH: ") << line << endl;
    if (match)
    {
        cout << "LAT: " << gps.latDegrees << "d " << gps.latArcMinutes
             << " " << (gps.north ? "NORTH" : "SOUTH") << endl;
        cout << "LONG: " << gps.lonDegrees << "d " << gps.lonArcMinutes
             << " " << (gps.east ? "EAST" : "WEST") << endl;
    }
}

// simple wrapper for a ComPort so that you can easily read character
// by character and "peek" at the next character Note that
// byte-by-byte is a really inefficient way to read, but until someone
// convices me that this is actually a bottle-neck, I can't be
// bothered to fix it :)
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

// Convert degrees (in the format that we get raw from the GPS) into radians
double degreesToRadians(int degrees, double arcMinutes, bool positive)
{
    double totalDegrees = degrees + (arcMinutes/60.0);
    if (!positive)
    {
        totalDegrees *= -1;
    }

    return totalDegrees / 180.0 * PI;
}

void publishGpsInfo(const ros::Publisher& pub, const GpsInfo& info)
{
    double latitude = degreesToRadians(info.latDegrees, info.latArcMinutes, info.north);
    double longitude = degreesToRadians(info.lonDegrees, info.lonArcMinutes, info.east);
    
    geometry_msgs::Point32 msg;
    msg.x = latitude;
    msg.y = longitude;

    pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps");
    ros::NodeHandle n;
    ros::NodeHandle privateHandle("~");

    string portName;
    privateHandle.param<string>("port", portName, "/dev/ttyUSB0");

    ros::Publisher gpsPub = n.advertise<geometry_msgs::Point32>("gps_data", 1000);

    // connect to gps	
    ComPortHandle port = -1; //initializing com port handle

    ros::Rate gpsCheckRate(1); // once a second (1 Hz)
    while (port == -1 && ros::ok()) {
        port = OpenComPort(portName.c_str(), false);
        if (port == -1) {
            ROS_FATAL("[GPS] Unable to connect to GPS!");
        } else {
            ROS_INFO("[GPS] Connected to GPS!");
        }
        gpsCheckRate.sleep();
    }

    ComStream strm(port);
    string line;

    while (ros::ok()) {
        char character=strm.next();

        if (character!='\n'){
            line += character;
        } else {                
            GpsInfo gps;
            bool match = getLatLongFromLine(line, gps);

            if (match)
            {
                publishGpsInfo(gpsPub, gps);
            }
                    
            // For debugging
            printMatch(match, line, gps);
                
            line.clear();
        }	
    }
	
    return 0;
}

