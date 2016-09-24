#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <queue>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include "utils/EvalUtils.hpp"
#include "lib_flag_detection.hpp"

using namespace cv;
using namespace std;


// You would invoke at the command line like: command_name <path_to_video_file>
int main(int argc, char** argv)
{

    // Expects one arg (argc is 2 because it also counts the command itself)
    if (argc < 2)
    {
        cout << "usage to enable normal behaviour: ./flag_detection <Video_Path>" << endl;
        cout << "usage for testing with pictures: ./flag_detection <Video_Path> true" << endl;
        return -1;
    }

    EvalUtils evalUtils;

    if(argc == 2){
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

    // Just loop forever, reading in frames from the video file. This eventually freezes up and crashes
    // when we hit the end of the video file. Oh well :)
    while (true)
    {
        evalUtils.startMainClock();

        // Was really slow for me, so to make it slightly less painful I read in 3 frames at a time and
        // skip processing on 2 out of the 3.
        Mat frame;
        
        // Also possible in video mode is to use evalUtils.getFrame(frame, 3); to skip frames
        frame = evalUtils.getFrame();//cap >> frame;

     	std::vector<blob> redFlags;
     	std::vector<blob> blueFlags;

		extractFlagsFromImage(frame, redFlags, blueFlags);

        // Expects an image with one color channel
        //evalUtils.writeImageOutput(distRed);

        // Draw red rectangles around each of the blobs
        for (int i = 0; i < redFlags.size(); i++)
        {
            blob b = redFlags[i];
            rectangle(frame, Point(b.minj, b.mini), Point(b.maxj, b.maxi), Scalar(0, 0, 255), 5);
        }

        // Show the original image with boxes around the flags
        imshow("original", frame);

        // Either wait 30ms and continue the loop, or, if the user presses a key and waitKey returns positive,
        // then break out of the loop and end the program.
        if (waitKey(30) >= 0)
        {
            break;
        }
        
        evalUtils.endMainClock();
    }  

    return 0;
}