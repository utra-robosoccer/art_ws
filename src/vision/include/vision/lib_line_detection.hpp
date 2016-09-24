#pragma once

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

void extractLinesFromImage(cv::Mat& frame, std::vector<cv::Vec2f> &transformedLines1, std::vector<cv::Vec2f> &transformedLines2, cv::Mat& output, cv::Mat& calibration, bool debug=false);
bool readMat(std::string fileName, cv::Mat& mat);

