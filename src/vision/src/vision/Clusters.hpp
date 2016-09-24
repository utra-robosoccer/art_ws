#pragma once

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <cmath>

// Yes, I did actually create an array of 56 distinct colours. What of it? :)
// PS this is from http://stackoverflow.com/questions/309149/generate-distinctly-different-rgb-colors-in-graphs
// And there's a pattern to them. So it's not like I (or anyone else) actually spent the time to come up with 56 distinct-looking colours :)
const cv::Scalar colours[] =  { 
    cv::Scalar(0xFF, 0x00, 0x00), cv::Scalar(0x00, 0xFF, 0x00), cv::Scalar(0x00, 0x00, 0xFF), cv::Scalar(0xFF, 0xFF, 0x00), cv::Scalar(0xFF, 0x00, 0xFF), cv::Scalar(0x00, 0xFF, 0xFF), cv::Scalar(0x00, 0x00, 0x00), 
    cv::Scalar(0x80, 0x00, 0x00), cv::Scalar(0x00, 0x80, 0x00), cv::Scalar(0x00, 0x00, 0x80), cv::Scalar(0x80, 0x80, 0x00), cv::Scalar(0x80, 0x00, 0x80), cv::Scalar(0x00, 0x80, 0x80), cv::Scalar(0x80, 0x80, 0x80), 
    cv::Scalar(0xC0, 0x00, 0x00), cv::Scalar(0x00, 0xC0, 0x00), cv::Scalar(0x00, 0x00, 0xC0), cv::Scalar(0xC0, 0xC0, 0x00), cv::Scalar(0xC0, 0x00, 0xC0), cv::Scalar(0x00, 0xC0, 0xC0), cv::Scalar(0xC0, 0xC0, 0xC0), 
    cv::Scalar(0x40, 0x00, 0x00), cv::Scalar(0x00, 0x40, 0x00), cv::Scalar(0x00, 0x00, 0x40), cv::Scalar(0x40, 0x40, 0x00), cv::Scalar(0x40, 0x00, 0x40), cv::Scalar(0x00, 0x40, 0x40), cv::Scalar(0x40, 0x40, 0x40), 
    cv::Scalar(0x20, 0x00, 0x00), cv::Scalar(0x00, 0x20, 0x00), cv::Scalar(0x00, 0x00, 0x20), cv::Scalar(0x20, 0x20, 0x00), cv::Scalar(0x20, 0x00, 0x20), cv::Scalar(0x00, 0x20, 0x20), cv::Scalar(0x20, 0x20, 0x20), 
    cv::Scalar(0x60, 0x00, 0x00), cv::Scalar(0x00, 0x60, 0x00), cv::Scalar(0x00, 0x00, 0x60), cv::Scalar(0x60, 0x60, 0x00), cv::Scalar(0x60, 0x00, 0x60), cv::Scalar(0x00, 0x60, 0x60), cv::Scalar(0x60, 0x60, 0x60), 
    cv::Scalar(0xA0, 0x00, 0x00), cv::Scalar(0x00, 0xA0, 0x00), cv::Scalar(0x00, 0x00, 0xA0), cv::Scalar(0xA0, 0xA0, 0x00), cv::Scalar(0xA0, 0x00, 0xA0), cv::Scalar(0x00, 0xA0, 0xA0), cv::Scalar(0xA0, 0xA0, 0xA0), 
    cv::Scalar(0xE0, 0x00, 0x00), cv::Scalar(0x00, 0xE0, 0x00), cv::Scalar(0x00, 0x00, 0xE0), cv::Scalar(0xE0, 0xE0, 0x00), cv::Scalar(0xE0, 0x00, 0xE0), cv::Scalar(0x00, 0xE0, 0xE0), cv::Scalar(0xE0, 0xE0, 0xE0), 
};

const int numColours = sizeof(colours) / sizeof(cv::Scalar);

// Class to represent a cluster of lines
class Cluster
{
public:
    void add(const cv::Vec4i& line);
    int getLineCount() const { return lineCount; }
    double getTotalDist() const { return totalDist; }
    double getLength() const { return length; }
    const std::vector<cv::Vec4i>& getLines() const { return lines; }

private:
    int lineCount; // How many lines in this cluster?
    double totalDist; // Sum of the length of all of the lines in the cluster?
    double length; // If you merged all the lines into one, how "long" would that one line be?
                   // Currently calculated as the maximum distance between any two points in any
                   // of the lines in the cluster
    std::vector<cv::Vec4i> lines; // An array of the actual lines in this cluster
};

double mag(const cv::Point2f& u);
cv::Point2f project(const cv::Point2f& u, const cv::Point2f& v);
double projectDist(const cv::Point2f& u, cv::Point2f& v);
bool linesMatch(const cv::Point2f& a1, const cv::Point2f& a2, const cv::Point2f& b1, const cv::Point2f& b2);
bool clusterGreaterThan(Cluster* a, Cluster* b);
void greedyClustering(const std::vector<cv::Vec4i>& lines, std::vector<int>& clusterAssignments, std::vector<Cluster*>& clusters);
