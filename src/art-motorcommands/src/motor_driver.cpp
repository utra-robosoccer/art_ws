#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <vector>
#include <math.h>
#include "visualization_msgs/Marker.h"

// Reference on ROS Node and Publisher:  http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

using namespace std;

#include "predefined_variables.h"       // contains variable that we need to calibrate
#include "Vector2.h"                    // class definition for 2D vector
#include "motor_command.h"              // contain defination of MotorCommand

// different approach: functions include in the following header files.
#include "move_and_rotate_command.h"    // move straight and turn way of motion
#include "curvatureApproach.h"      // path planning by fitting a circle to points, good for the case where waypoint has limited amount of noise and forms relatively smooth curves

ros::Publisher commandPub;
ros::Publisher waypointPublisher;

visualization_msgs::Marker createArrowMessage(const Vector2& start, const Vector2& end)
{    
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/actual-position";
    msg.ns = "motor_waypoint";
    msg.type = visualization_msgs::Marker::ARROW;
    msg.id = 0;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.x = 0.05; // arrow thickness
    msg.scale.y = 0.1; // arrow head thickness
    msg.color.b = 1; // color is blue
    msg.color.a = 1; // alpha is 100% (no transparency)

    geometry_msgs::Point startPoint, endPoint;
    double deltaX = end.x - start.x;
    double deltaY = end.y - start.y;
    double norm = sqrt(deltaX*deltaX + deltaY*deltaY);
    startPoint.x = 0;
    startPoint.y = 0;
    endPoint.x = deltaX / norm;
    endPoint.y = deltaY / norm;

    msg.points.push_back(startPoint);
    msg.points.push_back(endPoint);

    return msg;
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    cout << "Start processing a path!" << endl;

    vector<geometry_msgs::PoseStamped> points = msg->poses;
    vector<geometry_msgs::Point32> extracted_points;

    // Okay, so we've just received a new path. We extracted the points in that path.
    // Now we need to decide what to tell the robot to do.
    // We'd like to set leftMotorSpeed and rightMotorSpeed.

    // First let's just print the path.
    cout << "The number of points in the path is: " << points.size() << endl;

    // Next, iterate over all of the points and print each one
    for (int i = 0; i < points.size(); i++)
    {
        geometry_msgs::Point p = points[i].pose.position;

        // The coordinates of the point are stored in p.x, and p.y
        // Points are actually 3D, but we don't need p.z so we'll ignore it
        // These values are really floats (i.e. they are floating point numbers, with decimal points),
        // but we'll only ever use them as a grid position (so an integer). This means we can just cast them
        // to integers, as follows:
        int x = p.x;
        int y = p.y;

        geometry_msgs::Point32 ep;
        ep.x = x;
        ep.y = y;
        extracted_points.push_back(ep);

        // Now let's print a message saying where this point is:
        cout << "Point number " << i << " is at coordinates: (" << x << ", " << y << ")" << endl;
    }

    if (points.size() == 0)
    {
        cout << "Hmmm .... zero-length path! Ignoring!" << endl;
        return;
    }
    
    // Now the part where we actually have to decide what to tell the motors!
    // Do something to set leftMotorSpeed and rightMotorSpeed
    // <Insert code here>

    vector<Vector2> processedPath = getPaths (extracted_points);
    //vector<MotorCommand> commandList = calculateCommand (processedPath);
    vector<MotorCommand> commandList = deadSimpleCalculateCommand (processedPath);
    
    // Output a visualization message showing *just* the first message
    if (processedPath.size() >= 2)
    {
        waypointPublisher.publish(createArrowMessage(processedPath[0], processedPath[1]));
    }
    
    // Yeah, this is a really dumb way to do things. It's temporary, okay?
    // Sue me!
    ostringstream commandStream;
    
    // print the command list
    for (int i=0; i<commandList.size(); i++)
    {
        cout << "Command " << i << " :" << 
        " L " << commandList[i].leftSpeed <<
        " R " << commandList[i].rightSpeed <<
        " T " << commandList[i].timeInterval << endl;

        commandStream
            << "L " << commandList[i].leftSpeed
            << " R " << commandList[i].rightSpeed
            << " T " << commandList[i].timeInterval
            << ";";
    }

    cout << "The full command is the following: " << commandStream.str() << endl;
    
    std_msgs::String string_msg;
    string_msg.data = commandStream.str();
    commandPub.publish(string_msg);
}

// Some hacky magic to make C++ not suck ... What else is new?
template <size_t N> geometry_msgs::Polygon makePolygon (const int (&data)[N][2])
{
    vector<geometry_msgs::Point32> v;
    for (size_t i = 0; i < N; i++)
    {
        geometry_msgs::Point32 p;
        p.x = data[i][0];
        p.y = data[i][1];
        v.push_back(p);
    }
    
    geometry_msgs::Polygon poly;
    poly.points = v;

    return poly;
}

// main is what gets called when the program starts up
int main(int argc, char** argv)
{
    // Setup this ros node
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle n;

    // Subscribe to path data
    ros::Subscriber sub = n.subscribe("path", 1000, pathCallback);

    // Publish a visualization message to show the first waypoint in the path
    waypointPublisher = n.advertise<visualization_msgs::Marker>("waypoint", 1000);
    
    // We also (for testing purposes) want to publish path data!
    // This lets us generate fake paths for testing
    //ros::Publisher pub = n.advertise<geometry_msgs::Polygon>("path", 1000);

    // For publishing motor commands
    commandPub = n.advertise<std_msgs::String>("motor_commands", 1000);

    // For now, let's just publish a bunch of fake paths for our code to process
    int path1[][2] = {{0,10}, {1,11}, {0,12}};
    //pub.publish(makePolygon(path1));

    // Tell ROS to process any incoming messages (like the path we just published!)
    ros::spinOnce();

    // Another fake path
    int path2[][2] = {{0,0}, {1,1}, {2,2}, {3,3}, {4,4}, {5,5}, {6,6}, {7,7}, {8,8}};
    //pub.publish(makePolygon(path2));
    
    // Tell ROS to process any incoming messages (like the path we just published!)
    ros::spinOnce();

    // Another fake path
    int path3[][2] = {{-5,5}, {-4,6}, {-4,7}, {-4,8}, {-4,9}, {-5,10}, {-6,10}, {-7,10}, {-8,10}, {-9,10}};
    //pub.publish(makePolygon(path3));
    ros::spinOnce();

    // ros::Rate r(0.1);
    // while (ros::ok())
    // {
    
    //     // Another fake path
    //     int path4[][2] = {{1,1}, {1,2}, {2,2}, {2,3}, {3,4}, {4,5}, {3,5}, {2,6}, {3,6}, {4,6}, {5,6}, {5,5}, {5,4}, {6,3}, {7,2}, {7,1}, {7,0}, {1,1}};
    
    //     //pub.publish(makePolygon(path4));
    
    //     // Tell ROS to process any incoming messages (like the path we just published!)
    //     ros::spinOnce();

    //     r.sleep();
    // }

    ros::spin();
    
    // // go straight and turn method
    // vector<Vector2> processedPath = getPaths (path4);
    // vector<MotorCommand> commandList = calculateCommand (processedPath);
    
    // // print the command list
    // for (int i=0; i<commandList.size(); i++)
    // {
    //     cout << "Command " << i << " :" << 
    //     " L " << commandList[i].leftSpeed <<
    //     " R " << commandList[i].rightSpeed <<
    //     " T " << commandList[i].timeInterval << endl; 
    // }
    
    // // Circle fitting command generation
    // commandList = calculate_command_base_on_best_fit_circle (processedPath);
    // // print the command list
    // for (int i=0; i<commandList.size(); i++)
    // {
    //     cout << "Command " << i << " :" << 
    //     " L " << commandList[i].leftSpeed <<
    //     " R " << commandList[i].rightSpeed <<
    //     " T " << commandList[i].timeInterval << endl; 
    // }
    
    
    // This tells ROS to just continually process incoming messages
    // So we'll process the path we just published, and then stall forever
    // ros::spin();
    
    return 0;
}


































