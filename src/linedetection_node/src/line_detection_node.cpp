#include <iostream>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include "std_msgs/Int32.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "linedetection_node.h"

using namespace std;
using namespace cv;

void publishLineInfo(const ros::Publisher& pub, std::vector<cv::Vec2f> l1)
{
 
    geometry_msgs::Polygon msg;
    geometry_msgs::Point32 point;

    // http://answers.ros.org/question/60614/how-to-publish-a-vector-of-unknown-length-of-structs-in-ros/
    for(int i = 0 ; i < l1.size(); i++){
        point.x = l1[i][0];
        point.y = l1[i][1];
//        cout  << "Added " << point.x << " " << point.y;
        // http://answers.ros.org/question/145086/how-to-correctly-declare-a-polygon-message/
        msg.points.push_back(point);
    }
    
    pub.publish(msg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "imgproc_node");
    ros::NodeHandle n;
    ros::NodeHandle privateHandle("~");

    string videoFile, calibrationFile;
    privateHandle.param<string>("file", videoFile, "<camera>");
    privateHandle.param<string>("calibration", calibrationFile, "");

    if (calibrationFile == "")
    {
        cout << "Oops, no calibration file specified. Use command line option _calibration:=filename" << endl;
        return -1;
    }

    Mat calibration(3, 3, CV_64F);
    bool success = readMat(calibrationFile, calibration);
    if (!success) {
        cout << "Oops, problem reading calibration file" << endl;
        return -1;
    }

    cout << "Using the following calibration matrix: " << calibration << endl;
    
    bool recordInput;
    privateHandle.param("record", recordInput, true);

    bool showImage;
    privateHandle.param("show", showImage, true);

    VideoCapture cap;
    if (videoFile == "<camera>")  {
        cap.open(0);
    } else {
        cap.open(videoFile);
    }

    if(!cap.isOpened()) {
        cout << "Oops, couldn't open " << (videoFile == "<camera>" ? "camera feed" : "file: " + videoFile) << endl;
        return -1;
    }

    // If applicable, open a video writer to record our input
    VideoWriter outputVideo;
    if (recordInput) {
        Size size = Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
        const string name = "outputVideo.avi";
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        int fps = 15;
        outputVideo.open(name, codec, fps, size);

        if (!outputVideo.isOpened()) {
            cout << "Oops, couldn't open output file: " + name << endl;
            return -1;
        }
    }

    ros::Publisher linesPub = n.advertise<geometry_msgs::Polygon>("cam_data", 1000);
    ros::Publisher readFramePub = n.advertise<std_msgs::Int32>("read_frame", 1000);

    // Create a matrix for showing debug output
    Mat outputImage;

    // Count the number of frames we've seen so far
    int counter = 0;
    while (ros::ok()) {

        // Get camera frame from webcam
        cv::Mat frame;
        cap >> frame;

        if (frame.rows == 0 || frame.cols == 0)
        {
            cout << "Oops, got a bad frame. Skipping" << endl;
            continue;
        }
        else
        {
            cout << frame.rows << " by " << frame.cols << endl;
        }

        // Check if we're supposed to be recording frames
        if (recordInput) {
            // Send a signal that we just read a frame, so that other sensors can synchronize their readings
            std_msgs::Int32 msg;
            msg.data = counter;
            readFramePub.publish(msg);
            counter++;

            // Actually record the frame
            outputVideo << frame;
        }

        std::vector<cv::Vec2f> transformedLines1;
        std::vector<cv::Vec2f> transformedLines2;

        extractLinesFromImage(frame, transformedLines1, transformedLines2, outputImage, calibration);
    
        if (showImage) {
            imshow("line detection", outputImage);
            imshow("original", frame);
            waitKey(5); // It seems like you need this if you want the image to show up ...
        }
    
        // Concatenate the two vectors
        std::vector<cv::Vec2f> AB;
        AB.insert(AB.end(), transformedLines1.begin(), transformedLines1.end());
        AB.insert(AB.end(), transformedLines2.begin(), transformedLines2.end());
        // http://stackoverflow.com/questions/3177241/best-way-to-concatenate-two-vectors
        publishLineInfo(linesPub, AB);

    }
    
    return 0;
}

