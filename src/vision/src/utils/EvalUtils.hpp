#ifndef EVALUTILS_HPP
#define EVALUTILS_HPP

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <queue>
#include <ctime>

using namespace cv;
using namespace std;
class EvalUtils{
	public:
		int openInputFeed(bool isVideoFeed, char* fileName = NULL);
		int openImageFeed(int num_images);
		int openVideoFeed(char* fileName);
		cv::Mat getFrame(int numFrameSkips=1);
		cv::Mat getImageFrame();
		void writeImageOutput(cv::Mat frameToSave);
		void setupCommandLine(int argc, char** argv);
		void startMainClock();
		void endMainClock();
	private:
		cv::VideoCapture* mVideoCapture;
		std::string* mImageList;
		int mImageCounter;
		bool mIsVideoFeed;
		std::string mCurrentImageName;
		std::clock_t mStart;
		int mFrameCounter;	// For displaying average fps in video
		double mTotalTime;

};


#endif // EVALUTILS_HPP