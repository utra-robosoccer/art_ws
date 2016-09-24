#ifndef __LIB_FLAG_DETECTION_HPP__
#define __LIB_FLAG_DETECTION_HPP__

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

struct blob
{
    int index;
    int pixelCount;
    int mini, maxi, minj, maxj;
};

void extractFlagsFromImage(cv::Mat &frame, std::vector<blob> &redFlags, std::vector<blob> &blueFlags);

#endif
