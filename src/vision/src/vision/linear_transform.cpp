#include "linear_transform.hpp"
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <stdio.h>
#include <fstream>

using namespace std;
using namespace cv;

void warpPerspectiveClusters(Mat& calibration, Mat source, Cluster* c1, Cluster* c2, 
    std::vector<cv::Vec2f>& c1Outf,
    std::vector<cv::Vec2f>& c2Outf){

    // Set numbers here
    int alpha_=30, beta_=90., gamma_=90.;
    int f_ = 500, dist_ = 500;

    Mat destination;
    
    double f, dist;
    double alpha, beta, gamma;
    alpha = ((double)alpha_ - 90.)*CV_PI/180;
    beta = ((double)beta_ - 90.)*CV_PI/180;
    gamma = ((double)gamma_ - 90.)*CV_PI/180;
    f = (double) f_;
    dist = (double) dist_;

    Size taille = source.size();

    double w = (double)taille.width, h = (double)taille.height;


    // Projection 2D -> 3D matrix
    Mat A1 = (Mat_<double>(4,3) <<
        1, 0, -w/2,
        0, 1, -h/2,
        0, 0,    0,
        0, 0,    1);

    // Rotation matrices around the X,Y,Z axis
    Mat RX = (Mat_<double>(4, 4) <<
        1,          0,           0, 0,
        0, cos(alpha), -sin(alpha), 0,
        0, sin(alpha),  cos(alpha), 0,
        0,          0,           0, 1);

    Mat RY = (Mat_<double>(4, 4) <<
        cos(beta), 0, -sin(beta), 0,
                0, 1,          0, 0,
        sin(beta), 0,  cos(beta), 0,
                0, 0,          0, 1);

    Mat RZ = (Mat_<double>(4, 4) <<
        cos(gamma), -sin(gamma), 0, 0,
        sin(gamma),  cos(gamma), 0, 0,
        0,          0,           1, 0,
        0,          0,           0, 1);

    // Composed rotation matrix with (RX,RY,RZ)
    Mat R = RX * RY * RZ;

    // Translation matrix on the Z axis change dist will change the height
    Mat T = (Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dist,
        0, 0, 0, 1);

    // Camera Intrisecs matrix 3D -> 2D
    Mat A2 = (Mat_<double>(3,4) <<
        f, 0, w/2, 0,
        0, f, h/2, 0,
        0, 0,   1, 0);

    // Final and overall transformation matrix
    //Mat transfo = A2 * (T * (R * A1));

    // Mat transfo = (Mat_<double>(3, 3) <<
    //                -0.08599313400332668, -0.008699359415207681, 28.08139058790451,
    //                -0.003448729864241165, 0.115344364379626, -103.4293067411731,
    //                0.0001256075443696381, -0.007369227480602647, 1);

    // Mat transfo = (Mat_<double>(3, 3) <<
    //                -0.7383747692040197, 0.08073446207016484, 191.4580192531871,
    //                -0.02192662989642821, 1.167252049453472, -989.8113837496418,
    //                -0.002950898793956765, -0.04770615625313716, 1);

    // Mat transfo = (Mat_<double>(3, 3) <<
    //                0.2097471961580797, -0.123616200897436, -51.88254706769077,
    //                -0.07193545218048962, 1.000187117227966, 102.649627818762,
    //                -0.001357096684163634, 0.03715312159676666, 1);

    // Apply matrix transformation to the vectors
    // perspectiveTransform operatoes on vectors

    // Convert the Vec4i type to Vec2f
    std::vector<cv::Vec4i> c1Outi = c1->getLines();
    std::vector<cv::Vec2f> c1Inf;

    std::vector<cv::Vec4i> c2Outi = c2->getLines();
    std::vector<cv::Vec2f> c2Inf;

    for (int i = 0 ; i < c1->getLineCount(); i++){

        cv::Vec2f v1(c1Outi[i][0], c1Outi[i][1]);
        cv::Vec2f v2(c1Outi[i][2], c1Outi[i][3]);
        
        c1Inf.push_back(v1);
        c1Inf.push_back(v2);
    }
    cout << endl;

    for(int i = 0; i < c2->getLineCount(); i++){

        cv::Vec2f v1((float) c2Outi[i][0], (float) c2Outi[i][1]);
        cv::Vec2f v2((float) c2Outi[i][2], (float) c2Outi[i][3]);
        
        c2Inf.push_back(v1);
        c2Inf.push_back(v2);   
    }


    // Initialize output vector lengths

    perspectiveTransform(c1Inf, c1Outf, calibration );
    perspectiveTransform(c2Inf, c2Outf, calibration );    
    
    //Mat tmp;
    //warpPerspective(source, tmp, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

    //imshow("tmp", tmp);    

    return;
}


