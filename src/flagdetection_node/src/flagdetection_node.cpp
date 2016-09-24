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
#include "flagdetection_node/flagBox.h"
#include "flagdetection_node/flagBoxArray.h"

#include "flagdetection_node/flagdetection_node.h"

using namespace std;
using namespace cv;

void publishFlagInfo(const ros::Publisher& pub, std::vector<blob> flags)
{
	// http://wiki.ros.org/msg#Building_.msg_Files
    flagdetection_node::flagBox box;
    flagdetection_node::flagBoxArray fs;

    // http://answers.ros.org/question/60614/how-to-publish-a-vector-of-unknown-length-of-structs-in-ros/
    for(int i = 0 ; i < flags.size(); i++){
		box.mini = flags[i].mini;
		box.maxi = flags[i].maxi;
		box.minj = flags[i].minj;
		box.maxj = flags[i].maxj;
		fs.flags.push_back(box);
    }
	pub.publish(fs);	
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "flagdetection_node");
    ros::NodeHandle n;
    ros::NodeHandle privateHandle("~");

    string videoFile;
    privateHandle.param<string>("file", videoFile, "<camera>");

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
        const string name = "FlagVout.avi";
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        int fps = 15;
        outputVideo.open(name, codec, fps, size);

        if (!outputVideo.isOpened()) {
            cout << "Oops, couldn't open output file: " + name << endl;
            return -1;
        }
    }

	// depending on the decision, we may have a red and a blue flag topic or just a flag topic.
    ros::Publisher redFlagsPub = n.advertise<flagdetection_node::flagBoxArray>("cam_data_red_flags", 1000);
    ros::Publisher blueFlagsPub = n.advertise<flagdetection_node::flagBoxArray>("cam_data_blue_flags", 1000);
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

        std::vector<blob> redFlags;
        std::vector<blob> blueFlags;

        extractFlagsFromImage(frame, redFlags, blueFlags);

        if (showImage) {
            imshow("flag detection", outputImage);
            imshow("original", frame);
            waitKey(5); // It seems like you need this if you want the image to show up ...
        }
	
        publishFlagInfo(redFlagsPub, redFlags);
        publishFlagInfo(blueFlagsPub, blueFlags);

    }

    return 0;
}
