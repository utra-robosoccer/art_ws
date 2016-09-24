#include "EvalUtils.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <queue>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <ctime>


using namespace cv;
using namespace std;

int EvalUtils::openInputFeed(bool isVideoFeed, char* fileName){
    mIsVideoFeed = isVideoFeed;

    if(mIsVideoFeed){
        return(this->openVideoFeed(fileName));
    }else{
        return(this->openImageFeed(100));
    }

}

int EvalUtils::openVideoFeed(char* fileName){

    std::string file(fileName);

    // Start streaming in the video file
    mVideoCapture = new VideoCapture(file);

    // Initialize values for FPS calculation
    mTotalTime = 0;    
    mFrameCounter = 0;

    // Verify that the video is opened, crash if it is not
    if (!mVideoCapture->isOpened())
    {
        return(-1);
    }

    return(0);
}

cv::Mat EvalUtils::getFrame(int numFrameSkips){
    Mat frame;

    // Check whether in video feed mode or image mode
    if(mIsVideoFeed){   // Get frame from video
    	for(int i = 0 ; i < numFrameSkips; i++){
    		*mVideoCapture >> frame;
    	}
    }else{  // Get next image
        frame = imread(mImageList[mImageCounter].c_str(), CV_LOAD_IMAGE_COLOR);
        mCurrentImageName = mImageList[mImageCounter];
        mImageCounter ++;
    }

	return (frame);
}

// Initializes the list of images to read
int EvalUtils::openImageFeed(int num_images){

    mImageCounter = 0;
    ifstream inputListFile;
    mImageList = new string[num_images];

    inputListFile.open("imageList.txt");

    // Read values from file
    if(inputListFile.is_open()){
        for(int i = 0 ; i < num_images && !inputListFile.eof(); i++){
            inputListFile >> mImageList[i];
            cout << mImageList[i] << endl;
        }
    }else{
        cout << "No imageList.txt to read" << endl;
    }


    return(0);

}

void EvalUtils::writeImageOutput(Mat frameToSave){
    if(mIsVideoFeed){

    }else{
        frameToSave.convertTo(frameToSave, CV_8UC3, 255);
        string output_filename = mCurrentImageName.erase(mCurrentImageName.length() -4) + "_ANSWER.jpg";
        cv::imwrite(output_filename, frameToSave);
        cout << "Saved frame " << output_filename << endl;
    }
}

void EvalUtils::startMainClock(){
    this->mStart = std::clock();
}

void EvalUtils::endMainClock(){

    double timeToProcessFrame = (std::clock() - mStart)/(double) CLOCKS_PER_SEC;
    mFrameCounter++;
    mTotalTime +=  timeToProcessFrame;
    cout << "Time for this frame is " << timeToProcessFrame << " average time is " << mTotalTime/mFrameCounter << endl;
    
  
}



