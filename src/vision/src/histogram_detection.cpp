#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "utils/EvalUtils.hpp"
#include "utils/Timer.hpp"

using namespace cv;
using namespace std;

template <typename _Tp>
void OLBP_(const Mat& src, Mat& dst) {
    dst = Mat::zeros(src.rows-2, src.cols-2, CV_8UC1);
    for(int i=1;i<src.rows-1;i++) {
        for(int j=1;j<src.cols-1;j++) {
            _Tp center = src.at<_Tp>(i,j);
            unsigned char code = 0;
            code |= (src.at<_Tp>(i-1,j-1) > center) << 7;
            code |= (src.at<_Tp>(i-1,j) > center) << 6;
            code |= (src.at<_Tp>(i-1,j+1) > center) << 5;
            code |= (src.at<_Tp>(i,j+1) > center) << 4;
            code |= (src.at<_Tp>(i+1,j+1) > center) << 3;
            code |= (src.at<_Tp>(i+1,j) > center) << 2;
            code |= (src.at<_Tp>(i+1,j-1) > center) << 1;
            code |= (src.at<_Tp>(i,j-1) > center) << 0;
            dst.at<unsigned char>(i-1,j-1) = code;
        }
    }
}

template <typename _Tp>
void ELBP_(const Mat& src, Mat& dst, int radius, int neighbors) {
    neighbors = max(min(neighbors,31),1); // set bounds...
    // Note: alternatively you can switch to the new OpenCV Mat_
    // type system to define an unsigned int matrix... I am probably
    // mistaken here, but I didn't see an unsigned int representation
    // in OpenCV's classic typesystem...
    dst = Mat::zeros(src.rows-2*radius, src.cols-2*radius, CV_32SC1);
    for(int n=0; n<neighbors; n++) {
        // sample points
        float x = static_cast<float>(radius) * cos(2.0*M_PI*n/static_cast<float>(neighbors));
        float y = static_cast<float>(radius) * -sin(2.0*M_PI*n/static_cast<float>(neighbors));
        // relative indices
        int fx = static_cast<int>(floor(x));
        int fy = static_cast<int>(floor(y));
        int cx = static_cast<int>(ceil(x));
        int cy = static_cast<int>(ceil(y));
        // fractional part
        float ty = y - fy;
        float tx = x - fx;
        // set interpolation weights
        float w1 = (1 - tx) * (1 - ty);
        float w2 =      tx  * (1 - ty);
        float w3 = (1 - tx) *      ty;
        float w4 =      tx  *      ty;
        // iterate through your data
        for(int i=radius; i < src.rows-radius;i++) {
            for(int j=radius;j < src.cols-radius;j++) {
                float t = w1*src.at<_Tp>(i+fy,j+fx) + w2*src.at<_Tp>(i+fy,j+cx) + w3*src.at<_Tp>(i+cy,j+fx) + w4*src.at<_Tp>(i+cy,j+cx);
                // we are dealing with floating point precision, so add some little tolerance
                dst.at<unsigned int>(i-radius,j-radius) += ((t > src.at<_Tp>(i,j)) && (abs(t-src.at<_Tp>(i,j)) > std::numeric_limits<float>::epsilon())) << n;
            }
        }
    }
}

// http://stackoverflow.com/questions/24930134/entropy-for-a-gray-image-in-opencv
float calculateEntropy(cv::Mat mat){
    // Establish the number of bins
    int histSize = 256;
    // Set the ranges ( for B,G,R) )
    cv::Mat hist;
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;

    // Compute the histograms:
    calcHist( &mat, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
    hist /= mat.total();

    Mat logP;
    cv::log(hist,logP);

    float entropy = -1*sum(hist.mul(logP)).val[0];

    return entropy;
}

// You would invoke at the command line like: command_name <path_to_video_file>
int main(int argc, char** argv)
{
    // Expects one arg (argc is 2 because it also counts the command itself)
    if (argc < 2)
    {
        cout << "usage to enable normal behaviour: ./histogram_detection <Video_Path>" << endl;
        cout << "usage for testing with pictures: ./histogram_detection <Video_Path> true" << endl;
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

    int patch_width = 11;
    int patch_height = 11;

    // Just loop forever, reading in frames from the video file. This eventually freezes up and crashes
    // when we hit the end of the video file. Oh well :)
    while (true)
    {

        Mat frame;
        frame = evalUtils.getFrame();
        evalUtils.startMainClock();

        //resize(frame, frame, Size(0,0), 0.5,0.5);
        cv::GaussianBlur(frame, frame, Size(19,19),1,1);

        // Split the image into RGB channels
        vector<Mat> channels;
        split(frame, channels);
        Mat blue = channels[0];
        Mat green = channels[1];
        Mat red = channels[2];

        cv::Mat grey;
        cv::cvtColor(frame,grey,CV_BGR2GRAY); 
        cv::Mat entropyImage (frame.rows, frame.cols, CV_32F);

        cv::Rect imageRect = cv::Rect(0, 0, frame.cols-1, frame.rows-1);

        int frame_width = frame.cols;
        int frame_height = frame.rows;

/*
        for(int j = 0 ; j < frame_height; j++){
            for(int i = 0 ; i < frame_width; i++){
                cv::Rect a = cv::Rect(i, j, patch_width, patch_height);
                cv::Rect intersect = a & imageRect;
                cv::Mat targetRect = grey(intersect);
                entropyImage.at<float>(j,i) = calculateEntropy(targetRect);
            }

        }
*/
        double min, max;
        minMaxIdx(entropyImage, &min, &max);
        entropyImage = entropyImage / (float) max;
        //cv::normalize(entropyImage, entropyImage);

        // Expects an image with one color channel
        //evalUtils.writeImageOutput(answer);
        int erosion_size = 3;
        Mat kernel = getStructuringElement( cv::MORPH_RECT,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
        cv::Mat binarypattern (frame.rows, frame.cols, CV_8UC1);
        OLBP_<char>(red, binarypattern);
        cv::dilate(binarypattern, binarypattern, kernel);
        cv::erode(binarypattern, binarypattern, kernel);
        //ELBP_<char>(red, binarypattern, 5, 3);

       // Blend the two images
       evalUtils.endMainClock();
       imshow("r", red); 
       imshow("g", green); 
       imshow("b", blue); 
       imshow("answer", binarypattern);

        // Either wait 30ms and continue the loop, or, if the user presses a key and waitKey returns positive,
        // then break out of the loop and end the program.
        if (waitKey(30) >= 0)
       {
            break;
       }

       //getchar();
    }  

    return 0;
}
