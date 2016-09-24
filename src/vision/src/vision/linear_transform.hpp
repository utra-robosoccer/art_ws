#pragma once

#include <opencv2/opencv.hpp>
#include "Clusters.hpp"

// http://stackoverflow.com/questions/6606891/opencv-virtually-camera-rotating-translating-for-birds-eye-view
// http://delivery.acm.org/10.1145/360000/356750/p465-carlbom.pdf?ip=138.51.181.143&id=356750&acc=ACTIVE%20SERVICE&key=FD0067F557510FFB.148C9AE997532579.4D4702B0C3E38B35.4D4702B0C3E38B35&CFID=593536176&CFTOKEN=85902621&__acm__=1414875280_fc713fc96085ed6586375f96c215ad70
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=findhomography#findhomography
// http://docs.opencv.org/modules/imgproc/doc/geometric_transformations.html

// warpPerspective

void warpPerspectiveClusters(cv::Mat& calibration, cv::Mat source, Cluster* c1, Cluster* c2,
    std::vector<cv::Vec2f>& c1Outf,
    std::vector<cv::Vec2f>& c2Outf);


