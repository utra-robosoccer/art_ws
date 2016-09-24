#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "utils/EvalUtils.hpp"
#include <cmath>
#include "vision/linear_transform.hpp"
#include "vision/Clusters.hpp"

// Where main body of function is located
#include "lib_line_detection.hpp"

using namespace cv;
using namespace std;

#define PI 3.14159265358979323846

// General note: adaptive thresholding is probably not the right tool for the job here.
// Maybe something more like a single threshold (or divide image up into (few) block and
// have separate thresholds for each), and you start with a high threshold, and if you
// don't see "enough" for some definition of "enough", then you lower?
// Or just actually have a global threshold but use real #s from Philip's Matlab experiments
// to choose good values. "Informed" thresholding. 
// Idk ... but it kind of produces sucky results in a lot of cases, and I think we
// can do better


// You would invoke at the command line like: command_name <path_to_video_file>
int main(int argc, char** argv)
{

    // Expects 2 args (argc is 3 because it also counts the command itself)
    if (argc < 3)
    {
        cout << "usage to enable normal behaviour: ./line_detection_thresholding <Video_Path> <calibration_file>" << endl;
        cout << "usage for testing with pictures: ./line_detection_thresholding <Video_Path> <calibration_file> true" << endl;
        return -1;
    }

    EvalUtils evalUtils;

    if(argc == 3){
        cout << "Init video feed. " << endl;
        // Initialize image list for evaluating, pass true for video feed, false for image feed  
        if(evalUtils.openInputFeed(true, argv[1])!= 0){
            cout << "Uh oh, couldn't open the video file for some reason!" << endl;
            return -1;
        }
    }else{
        cout << "Init image feed. " << endl;
        if(evalUtils.openInputFeed(false)!= 0){
            cout << "Uh oh, couldn't open the images file for some reason!" << endl;
            return -1;
        }
    }

    Mat calibration(3, 3, CV_64F);
    bool success = readMat(argv[2], calibration);
    if (!success) {
        cout << "Oops, problem reading calibration file" << endl;
        return -1;
    }

    // Just loop forever, reading in frames from the video file. This eventually freezes up and crashes
    // when we hit the end of the video file. Oh well :)
    while (true)
    {
        evalUtils.startMainClock();

        Mat frame;
        Mat out;
        
        // Also possible in video mode is to use evalUtils.getFrame(3); to skip frames
        frame = evalUtils.getFrame(2);//cap >> frame;

        std::vector<cv::Vec2f> transformedLines1;
        std::vector<cv::Vec2f> transformedLines2;


        extractLinesFromImage(frame, transformedLines1, transformedLines2, out, calibration, true);

        evalUtils.endMainClock();
        // Either wait 30ms and continue the loop, or, if the user presses a key and waitKey returns positive,
        // then break out of the loop and end the program.
        if (waitKey(30) >= 0)
       {
            break;
       }
    }  

    return 0;
}
