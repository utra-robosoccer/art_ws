#include "lib_line_detection.hpp"
#include "Clusters.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <cmath>
#include "linear_transform.hpp"

using namespace std;
using namespace cv;

// Pass everything by reference so don't make copy
// Use the debug boolean in code testing to display imshows
void extractLinesFromImage(cv::Mat& frame, std::vector<cv::Vec2f> &transformedLines1, std::vector<cv::Vec2f> &transformedLines2, cv::Mat& output, cv::Mat& calibration, bool debug){

    // Shrink by 2x in both directions
    //resize(frame, frame, Size(0, 0), 0.5, 0.5);

    // Split the image into red, green, blue channels.
    // We could potentially use some other colour system here (HSV?)
    vector<Mat> channels;
    split(frame, channels);
    Mat blue = channels[0];
    Mat green = channels[1];
    Mat red = channels[2];

    // Initialize some matrices which we'll use to store the distance (and square distance)
    // to white of each pixel.
    Mat dist = Mat::zeros(frame.rows, frame.cols, CV_32F); 
    Mat distSquare = Mat::zeros(frame.rows, frame.cols, CV_32F); 


    Mat blue_squared;
    Mat red_squared;
    Mat green_squared;
/**/
   // In order to do the multiply, need to convert the range from 0-255 to 32 bit
    blue.convertTo(blue, CV_32F);
    red.convertTo(red, CV_32F);
    green.convertTo(green, CV_32F);

    blue = 255 - blue;
    green = 255 - green;
    red = 255- red;        

    // cv::multiply(red, red, red_squared, 1/300, CV_32F); was saturating for some reason
    red_squared = red.mul(red);
    green_squared = green.mul(green);
    blue_squared = blue.mul(blue);


    // More tricky thresholding operations here
    // In the for loop, the calculated values for dSqr, d must be > 0 (since squaring)
    // So in the if statement, we are really just pushing everything saturated 
    // to 0 and everything else to 255 - dSqr, 255 - d
    // In the matrix operations, the distSquare is clamped to 0 <= x <= 255
    // Once we subtract, everything greater than 255 becomes 0, and everything that was 
    // originally less than zero is 255 -distSquare
    distSquare = (red_squared + blue_squared + green_squared)/300;
    distSquare.convertTo(distSquare, CV_8U);
    distSquare = 255-distSquare;    

    cv::sqrt((red_squared + green_squared + blue_squared), dist);      
    dist.convertTo(dist, CV_8U);        
    dist = 255 - dist;

/*     
    // Iterate over the image
    for (int i = 0; i < frame.rows; i++)
    {
        for (int j = 0; j < frame.cols; j++)
        {
            int b = 255 - blue.at<uchar>(i,j);
            int g = 255 - green.at<uchar>(i,j);
            int r = 255 - red.at<uchar>(i,j);

            // Calcualte distance to white and distance squared to white
            // For distance squared, we divide by 300 to keep the numbers small and manageable
            // This is mainly just so we can easily imshow the image.
            int dSqr = (b*b + g*g + r*r)/300;
            int d = sqrt(b*b + g*g + r*r);
            dist.at<uchar>(i,j) = d > 255 ? 0 : (255 - d);
            distSquare.at<uchar>(i,j) = dSqr > 255 ? 0 : (255 - dSqr);
        }
    }      
/**/

    // Blur the two distance images to get rid of noise
    GaussianBlur(dist, dist, Size(21,21), 5, 5);
    GaussianBlur(distSquare, distSquare, Size(21,21), 5, 5);

    // Do adaptive thresholding on the two distance images
    Mat thresh;
    Mat thresh2;
    adaptiveThreshold(dist, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 31, -5);
    adaptiveThreshold(distSquare, thresh2, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 31, -5);

    // Make a colour copy of each thresholded image. Everything currently in the image will remain in
    // grayscale (actaully, since it's a thresholded binary image, every pixel is either black or
    // white), but colour will allow us to superimpose colour lines overtop of the image later.
    Mat colorThresh, colorThreshBest;
    cvtColor(thresh, colorThresh, CV_GRAY2BGR); // This one will have all lines on it
    cvtColor(thresh, colorThreshBest, CV_GRAY2BGR); // This one will only have lines from the two best clusters

    Mat colorThresh2, colorThreshBest2;
    cvtColor(thresh2, colorThresh2, CV_GRAY2BGR); // This one will have all lines on it
    cvtColor(thresh2, colorThreshBest2, CV_GRAY2BGR); // This one will only have lines from the two best clusters

    // ****************************************
    // Do processing on thresh
    // ***************************************

    // Do Hough Line Transform on the images to find lines
    vector<Vec4i> lines;
    HoughLinesP(thresh, lines, 1, CV_PI/180, 50, 80, 10);

    //imshow("houghlines", lines);

    // Do greedy clustering on the lines we find
    vector<int> clusterAssignments;
    vector<Cluster*> clusters;
    greedyClustering(lines, clusterAssignments, clusters);

    // Sort clusters (by their length currently, but we can experiment with different measures of "realness")
    vector<Cluster*> sortedClusters(clusters);
    std::sort(sortedClusters.begin(), sortedClusters.end(), clusterGreaterThan);

    // Uh oh. We don't have enough colours to show all these clusters :(
    if (clusters.size() > numColours)
    {
        cout << "Well, this is embarassing :(" << endl;
        exit(-1);
    }

    // Draw all the clusters onto the image
    for (size_t i = 0; i < sortedClusters.size(); i++)
    {
        Cluster* c = sortedClusters[i];

        // Draw each line in the cluster
        const vector<Vec4i>& clusterLines = c->getLines();
        for (size_t j = 0; j < clusterLines.size(); j++)
        {
            Vec4i v=clusterLines[j];
            line (colorThresh, Point(v[0], v[1]), Point(v[2], v[3]), colours[i], 3, CV_AA);

            // Only draw the best two clusters on the colorThreshBest image
            if (i < 2)
            {
                line (colorThreshBest, Point(v[0], v[1]), Point(v[2], v[3]), colours[i], 3, CV_AA);
            }
        }
    }

    // ****************************************
    // End processing on thresh.
    // ***************************************

    // ****************************************
    // Do the same processing on thresh2
    // ***************************************

    // Do Hough Line Transform on the images to find lines
    vector<Vec4i> lines2;
    HoughLinesP(thresh2, lines2, 1, CV_PI/180, 50, 80, 10);

    // Do greedy clustering on the lines we find
    vector<int> clusterAssignments2;
    vector<Cluster*> clusters2;
    greedyClustering(lines2, clusterAssignments2, clusters2);

    // Sort clusters (by their length currently, but we can experiment with different measures of "realness")
    vector<Cluster*> sortedClusters2(clusters2);
    std::sort(sortedClusters2.begin(), sortedClusters2.end(), clusterGreaterThan);

    // Uh oh. We don't have enough colours to show all these clusters :(
    if (clusters2.size() > numColours)
    {
        cout << "Well, this is embarassing :(" << endl;
        exit(-1);
    }

    // Draw all the clusters onto the image
    for (size_t i = 0; i < sortedClusters2.size(); i++)
    {
        Cluster* c = sortedClusters2[i];
        const vector<Vec4i>& clusterLines = c->getLines();

        if (i < 2)
        {
            cout << "Cluster " << i << " has " << c->getLineCount() << " lines" << endl;

        }

        // Draw each line in the cluster
        for (size_t j = 0; j < clusterLines.size(); j++)
        {


            Vec4i v=clusterLines[j];

            line (colorThresh2, Point(v[0], v[1]), Point(v[2], v[3]), colours[i], 3, CV_AA);

            // Only draw the best two clusters on the colorThreshBest2 image
            if (i < 2)
            {
     //           cout << "o1 " << v[0] << " " << v[1] << " o2 " << v[2] << " " << v[3] << endl;
                line (colorThreshBest2, Point(v[0], v[1]), Point(v[2], v[3]), colours[i], 3, CV_AA);
            }
        }
    }

    // ****************************************
    // End processing on thresh2
    // ***************************************


    // Perform the homography
    if (sortedClusters2.size() >= 2){
        warpPerspectiveClusters(calibration, colorThreshBest2, sortedClusters2[0], sortedClusters2[1],
                                transformedLines1, transformedLines2);

       for(size_t i = 0; i < transformedLines1.size(); i+=2){



            line(colorThreshBest2, Point(transformedLines1[i][0], transformedLines1[i][1]),
                Point(transformedLines1[i+1][0], transformedLines1[i+1][1]),
                colours[10], CV_AA);

     //       cout << "v1 " << transformedLines1[i][0] << " " << transformedLines1[i][1]
     //           << " v2 " << transformedLines1[i+1][0] << " " << transformedLines1[i+1][1] << endl;
        }

       for(size_t j = 0; j < transformedLines2.size(); j+=2){
            line(colorThreshBest2, Point(transformedLines2[j][0], transformedLines2[j][1]),
                Point(transformedLines2[j+1][0], transformedLines2[j+1][1]),
                colours[10], CV_AA);
       }
    }
    
    // Show the original image, the distance image, the thresolded image with all clusters, and the thresholded image with the two best clusters
    // We are currently only showing thresh2 derivatives, not thresh, because thresh2 gives better results.

    if(debug){
        imshow("original", frame);
        imshow("whiteness", dist);
        imshow("thresh", colorThresh2);
        imshow("threshBest", colorThreshBest2);
    }
  
    // Return thresh2 as the final product
    output = colorThreshBest2;

    // Free the memory in the clusters.
    // It sucks that I have to remember to do this :(
    // Fuck you C++
    // Although I guess technically I could have used smart pointers if it really
    // bothered me that much.
    for (size_t i = 0; i < clusters.size(); i++)
    {
        delete clusters[i];
    }
    for (size_t i = 0; i < clusters2.size(); i++)
    {
        delete clusters2[i];
    }
}

// Reads a matrix in from a file
bool readMat(string fileName, Mat& mat)
{
    ifstream in(fileName.c_str());

    if (!in.good()) return false;
    
    for (int r = 0; r < mat.rows; r++)
    {
        string line;
        bool success = getline(in, line);

        if (!success) return false;

        stringstream strm(line);

        for (int c = 0; c < mat.cols; c++)
        {
            double val;
            strm >> val;

            if (strm.fail()) return false;

            mat.at<double>(r, c) = val;
        }
    }

    return true;
}
