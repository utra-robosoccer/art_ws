#include <iostream>
#include <fstream>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <sstream>
#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


struct GpsInfo
{
    double latitude;
    double longitude;

    GpsInfo(double lat, double lon)
        : latitude(lat), longitude(lon)
    { }

    GpsInfo()
        : latitude(0), longitude(0)
    { }

};


bool pointSelected = false;
bool leftClickEvent = false;
bool rightClickEvent = false;
bool originalMoveEvent = false;
bool finalMoveEvent = false;
int globalX = 0;
int globalY = 0;
VideoCapture cap;
GpsInfo origin;
double cellSize;
double bearing;

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

void CallBackFuncGetPoint(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
         //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          pointSelected = true;
          globalX = x;
          globalY = y;
     }
}

void CallBackFuncShowPointsOriginal(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
         leftClickEvent = true;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
         rightClickEvent = true;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
         originalMoveEvent = true;
         globalX = x;
         globalY = y;
     }
}

void CallBackFuncShowPointsFinal(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
         leftClickEvent = true;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
         rightClickEvent = true;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
         finalMoveEvent = true;
         globalX = x;
         globalY = y;
     }
}


void findBounds(const vector<Point2f>& points, double& xmin, double& xmax, double& ymin, double& ymax)
{
    xmin = xmax = ymin = ymax = 0;
    for (size_t i = 0; i < points.size(); i++)
    {
        xmin = std::min(xmin, (double)(points[i].x));
        ymin = std::min(ymin, (double)(points[i].y));
        xmax = std::max(xmax, (double)(points[i].x));
        ymax = std::max(ymax, (double)(points[i].y));
    }
}

Point2f getOrigin(cv::Mat& dstImage, const vector<Point2f>& input)
{
    double xmin, xmax, ymin, ymax;
    findBounds(input, xmin, xmax, ymin, ymax);

    int border = 20;

    if (xmin == xmax)
    {
        xmin -= 1;
        xmax += 1;
    }

    if (ymin == ymax)
    {
        ymin -= 1;
        ymax += 1;
    }

    int width = dstImage.cols;
    int height = dstImage.rows;

    double xrange = (xmax - xmin);
    double yrange = (ymax - ymin);

    double xscale = (width - 2 * border) / xrange;
    double yscale = (height - 2 * border) / yrange;

    return Point2f((0 - xmin) * xscale + border, height - ((0 - ymin) * yscale + border));
}


void getDstPoints(cv::Mat& dstImage, const vector<Point2f>& input, vector<Point2f>& output, bool getOrigin = false)
{
    double xmin, xmax, ymin, ymax;
    findBounds(input, xmin, xmax, ymin, ymax);

    int border = 20;

    if (xmin == xmax)
    {
        xmin -= 1;
        xmax += 1;
    }

    if (ymin == ymax)
    {
        ymin -= 1;
        ymax += 1;
    }

    int width = dstImage.cols;
    int height = dstImage.rows;

    double xrange = (xmax - xmin);
    double yrange = (ymax - ymin);

    double xscale = (width - 2 * border) / xrange;
    double yscale = (height - 2 * border) / yrange;

    for (size_t i = 0; i < input.size(); i++)
    {
        const Point2f& p =  input[i];
        Point2f scaled((p.x - xmin) * xscale + border, height - ((p.y - ymin) * yscale + border));
        //cout << "Pushed back: " << scaled.x << ", " << scaled.y << endl;
        output.push_back(scaled);
    }

    if (getOrigin)
    {
        const Point2f p(0, 0);
        Point2f scaled((p.x - xmin) * xscale + border, height - ((p.y - ymin) * yscale + border));
        output.push_back(scaled);
    }
}




Point2f pixelToPoint(int x, int y, double xmin, double ymin, double xscale, double yscale, double border, int height)
{
    return Point2f((x - border)/xscale + xmin, (height - y - border)/yscale + ymin);
}

void getPointToPixelTransform(cv::Mat& dstImage, const vector<Point2f>& input, double& xmin, double& ymin, double& xscale, double& yscale, double& border, int& height)
{
    double xmax, ymax;
    findBounds(input, xmin, xmax, ymin, ymax);

    border = 20;

    if (xmin == xmax)
    {
        xmin -= 1;
        xmax += 1;
    }

    if (ymin == ymax)
    {
        ymin -= 1;
        ymax += 1;
    }

    int width = dstImage.cols;
    height = dstImage.rows;

    double xrange = (xmax - xmin);
    double yrange = (ymax - ymin);

    xscale = (width - 2 * border) / xrange;
    yscale = (height - 2 * border) / yrange;
}


void addPointsToImage(cv::Mat& image, const vector<Point2f>& points, bool lastIsOrigin = false)
{
    for (size_t i = 0; i < points.size(); i++)
    {
        const Point2f& p = points[i];
        Point centre(p.x, p.y);

        int thickness = -1; // means the circle will be filled
        int lineType = 8;
        int radius = 5;
        const Scalar& colour = (lastIsOrigin && i == points.size() - 1) ? Scalar(255, 255, 255) : colours[i % numColours];

        circle(image, centre, radius, colour, thickness, lineType);
    }
}

void drawDstPoints(cv::Mat& output, const vector<Point2f>& points)
{
    vector<Point2f> scaledPoints;
    const bool getOrigin = true;
    const bool lastIsOrigin = true;

    getDstPoints(output, points, scaledPoints, getOrigin);
    addPointsToImage(output, scaledPoints, lastIsOrigin);
}


// Returns the distance forward and back to get *from* a *to* b
// Bearing of 0 is north, positive means turn due east, negative means turn due west
// Uses: http://www.movable-type.co.uk/scripts/latlong.html
// Uses the equirectangular approximation, since we're talking about extremely small distances
Point2f getDstPointFromGps(double imu, const GpsInfo& point, const GpsInfo& origin, double cellSize)
{
    // There might be some wrap-around issues here, but, like, we're not going to be anywhere
    // near Greenwich, so we should be fine :)
    // PS these values are in radians
    double latDelta = point.latitude - origin.latitude;
    double lonDelta = point.longitude - origin.longitude;
    double averageLat = (origin.latitude + point.latitude) / 2;

    const double radiusOfEarth = 6371000; // metres

    double x = latDelta; // This is actually up-down, but in the context of our north-is-zero coordinate system, it makes most sense to call this the x axis
    double y = lonDelta * cos(averageLat);

    double metres = sqrt(x*x + y*y) * radiusOfEarth;
    double bearing = atan2(y, x);
    
    // Take into account the fact that the robot is not necessarily facing north
    bearing -= imu;

    // Calculate how far up and to the right of the robot's current location the goal lies
    double metresUp = cos(bearing) * metres;
    double metresRight = sin(bearing) * metres;

    double cellsUp = (metresUp / cellSize);
    double cellsRight = (metresRight / cellSize);

    return Point2f(cellsRight, cellsUp);
}

void printMask(vector<uchar>& mask)
{
    for (int i = 0; i < mask.size(); i++)
    {
        cout << "Point " << i << ": " << (mask[i] == 0 ? "outlier" : "GOOD!") << endl;
    }
}

void getPoints(Mat& frame, vector<Point2f>& srcPoints, vector<Point2f>& dstPoints)
{
    //set the callback function for any mouse event
    setMouseCallback("original", CallBackFuncGetPoint, NULL);

    while (ros::ok()) {

        cout << "Add point (a) or done (d) ?" << endl;
        cout << "> ";

        string input;
        cin >> input;

        if (input == "a")
        {
            pointSelected = false;
            while (!pointSelected)
            {
                cap >> frame;
                if (frame.rows == 0 || frame.cols == 0) continue;
                addPointsToImage(frame, srcPoints);
                cv::Mat world = cv::Mat::zeros(frame.rows, frame.cols, frame.type());
                drawDstPoints(world, dstPoints);
                imshow("world", world);
                imshow("original", frame);
                waitKey(5);
            }

            srcPoints.push_back(Point2f(globalX, globalY));
            addPointsToImage(frame, srcPoints);
            cv::Mat world = cv::Mat::zeros(frame.rows, frame.cols, frame.type());
            drawDstPoints(world, dstPoints);
            imshow("world", world);
            imshow("original", frame);
            waitKey(5);

            cout << "Specify location of point via gps (g) or distance (d)? " << endl;
            cout << "> ";
            string location;
            cin >> location;

            if (location == "g")
            {
                double lat, lon;
                cout << "Latitude (radians): ";
                cin >> lat;
                cout << "Longitude (radians): ";
                cin >> lon;

                GpsInfo point(lat, lon);
                dstPoints.push_back(getDstPointFromGps(bearing, point, origin, cellSize));
            }
            else
            {
                double x, y;
                cout << "Distance forward (metres): ";
                cin >> y;
                cout << "Distance right (metres): ";
                cin >> x;

                dstPoints.push_back(Point2f(x/cellSize, y/cellSize));
            }
        }
        else if (input == "d")
        {
            break;
        }
    }
}

bool showResults(const string methodNames[], const Mat* methodPixelTransforms[], const Mat* methodPositionTransforms[], const vector<Point2f>& srcPoints, const vector<Point2f>& dstPoints, Mat& frame, Mat& output)
{
    pointSelected = false;
    int method = 0;

    cout << "Okay. Now showing you the results of the transform." << endl;
    cout << "Click on the image to toggle between homography-finding methods." << endl;
    cout << "Using " << methodNames[method] << " method!" << endl;

    namedWindow("output", 1);
    setMouseCallback("original", CallBackFuncShowPointsOriginal, NULL);
    setMouseCallback("output", CallBackFuncShowPointsFinal, NULL);

    leftClickEvent = false;
    rightClickEvent = false;
    originalMoveEvent = false;
    finalMoveEvent = false;

    while (true)
    {
        if (rightClickEvent)
        {
            return false;
        }

        if (leftClickEvent)
        {
            leftClickEvent = false;
            method = (method + 1) % 3;
            cout << "Using " << methodNames[method] << " method!" << endl;
        }

        const Mat& transformation = *(methodPixelTransforms[method]);

        cap >> frame;
        if (frame.rows == 0 || frame.cols == 0) continue;

        // Transform the main image
        warpPerspective(frame, output, transformation, frame.size(), INTER_CUBIC);
        addPointsToImage(frame, srcPoints);

        // Transform the src points
        vector<Point2f> transformedPoints;
        vector<Point2f> transformedPixels;

        perspectiveTransform(srcPoints, transformedPoints, transformation);
        Point2f origin = getOrigin(output, dstPoints);
        transformedPoints.push_back(origin);
        const bool lastIsOrigin = true;
        addPointsToImage(output, transformedPoints, lastIsOrigin);

        cv::Mat world = cv::Mat::zeros(frame.rows, frame.cols, frame.type());
        drawDstPoints(world, dstPoints);

        imshow("original", frame);
        imshow("output", output);
        imshow("world", world);

        if (originalMoveEvent)
        {
            originalMoveEvent = false;
            vector<Point2f> src;
            src.push_back(Point2f(globalX, globalY));
            vector<Point2f> dst;

            perspectiveTransform(src, dst, *(methodPositionTransforms[method]));

            cout << "(" << dst[0].x << ", " << dst[0].y << ")" << endl;
        }

        if (finalMoveEvent)
        {
            finalMoveEvent = false;

            double xmin, ymin, xscale, yscale, border;
            int height;
            getPointToPixelTransform(world, dstPoints, xmin, ymin, xscale, yscale, border, height);

            Point2f point = pixelToPoint(globalX, globalY, xmin, ymin, xscale, yscale, border, height);

            cout << "(" << point.x << ", " << point.y << ")" << endl;
        }

        if (waitKey(30) >= 0)
        {
            return true;
        }
    }   
}

string getDatetimeString()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);

    return string(buf);
}

void writeMat(ostream& out, Mat& mat)
{
    for (int r = 0; r < mat.rows; r++)
    {
        for (int c = 0; c < mat.cols; c++)
        {
            out << std::setprecision (15) << mat.at<double>(r, c) << " ";
        }

        out << endl;
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "calibration");
    ros::NodeHandle n;
    ros::NodeHandle privateHandle("~");

    double ransacThreshold;
    string outputFile;
    privateHandle.param("ransacThresh", ransacThreshold, 5.0);
    privateHandle.param<string>("outputFile", outputFile, "<datetime>");

    if (outputFile == "<datetime>") {
        outputFile = getDatetimeString();
    }

    ofstream outReg((outputFile + "_regular").c_str());
    if (!outReg.good()) {
        cout << "Oops, could not open the output file: '" << outputFile << "'" << endl;
    }
    ofstream outRans((outputFile + "_ransac").c_str());
    if (!outRans.good()) {
        cout << "Oops, could not open the output file: '" << outputFile << "'" << endl;
    }
    ofstream outLmeds((outputFile + "_lmeds").c_str());
    if (!outLmeds.good()) {
        cout << "Oops, could not open the output file: '" << outputFile << "'" << endl;
    }
    ofstream outDef("defaultCalibration");
    if (!outDef.good()) {
        cout << "Oops, could not open the output file: 'defaultCalibration'" << endl;
    }
    
    cap.open(0);

    if(!cap.isOpened()) {
        cout << "Oops, couldn't open camera feed!" << endl;
        return -1;
    }

    cout << "Enter IMU orientation (radians east of due north): ";
    cin >> bearing;

    cout << "Enter cell size (m/cell): ";
    cin >> cellSize;

    double lat, lon;
    cout << "Enter current GPS location ..." << endl;
    cout << "  Latitude (radians): ";
    cin >> lat;
    cout << "  Longitude (radians): ";
    cin >> lon;
    origin = GpsInfo(lat, lon);
    
    vector<Point2f> srcPoints;
    vector<Point2f> dstPoints;    

    //Create a window
    namedWindow("original", 1);

    cv::Mat frame;
    cv::Mat output;

    while (ros::ok())
    {
        getPoints(frame, srcPoints, dstPoints);

        // Get the pixels of the dst points
        vector<Point2f> dstPixels;
        const bool getOrigin = false;
        getDstPoints(frame, dstPoints, dstPixels, getOrigin);

        cout << "Given the following data:" << endl;
        cout << "Format: src point -> dst point [aka dst pixels]" << endl;

        for (int i = 0; i < srcPoints.size(); i++)
        {
            cout << "(" << srcPoints[i].x << ", " << srcPoints[i].y << ") -> ";
            cout << "(" << dstPoints[i].x << ", " << dstPoints[i].y << ") [aka ";
            cout << "(" << dstPixels[i].x << ", " << dstPixels[i].y << ")]" << endl;
        }
        
        cout << "COMPUTING HOMOGRAPHIES" << endl;

        int regularMethod = 0;
        cv::Mat regular = findHomography(srcPoints, dstPixels, regularMethod, 0);
        cv::Mat regularToCells = findHomography(srcPoints, dstPoints, regularMethod, 0);

        cout << "Regular method: " << regularToCells << endl;
        writeMat(outReg, regularToCells);
        writeMat(outDef, regularToCells);

        vector<uchar> ransacMask;
        cv::Mat ransac = findHomography(srcPoints, dstPixels, CV_RANSAC, ransacThreshold, ransacMask);
        cv::Mat ransacToCells = findHomography(srcPoints, dstPoints, CV_RANSAC, ransacThreshold, ransacMask);

        cout << "Ransac method: " << ransacToCells << endl;
        cout << "Mask:" << endl;
        printMask(ransacMask);
        writeMat(outRans, ransacToCells);
        
        vector<uchar> lmedsMask;
        cv::Mat lmeds = findHomography(srcPoints, dstPixels, CV_LMEDS, 0, lmedsMask);
        cv::Mat lmedsToCells = findHomography(srcPoints, dstPoints, CV_LMEDS, 0, lmedsMask);
    
        cout << "Lmeds Method: " << lmedsToCells << endl;
        cout << "Mask:" << endl;
        printMask(lmedsMask);
        writeMat(outLmeds, lmedsToCells);
        
        const string methodNames[] = {"regular", "ransac", "lmeds"};
        const Mat* methodPixelTransforms[] = { &regular, &ransac, &lmeds };
        const Mat* methodPositionTransforms[] = { &regularToCells, &ransacToCells, &lmedsToCells };

        bool done = showResults(methodNames, methodPixelTransforms, methodPositionTransforms, srcPoints, dstPoints, frame, output);

        if (done) break;
    }

    return 0;
}

