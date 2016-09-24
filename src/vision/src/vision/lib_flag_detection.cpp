#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>
#include <queue>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include "lib_flag_detection.hpp"

using namespace cv;
using namespace std;

typedef void imgTransform(Mat&);

// Find the blob that (start_i, start_j) is a part of.
// We search the input image to find the largest connected blob of pixels which are "on", starting at
// the point (start_i, start_j). We use 4-connectivity, which means each pixel is connected to the
// pixels above, below, left, and right (but not diagonals). All pixels in the blob we find will be
// set to zero in the input image. All pixels in the blob will be labeled with the specified index
// number in the output image. Finally, we'll return a description of the blob we found.
//
// TODO: this whole breadth-first-search song and dance should be replaced by disjoint sets.
// We have a disjoint-forest implementation from last year, we can just use that.
blob extractBlob(Mat& input, Mat& output, int start_i, int start_j, int index)
{
    // We're essentially doing a breadth-first search for all pixels connected to (start_i, start_j)

    // Create the queue of pixels to explore, starting with our start point
    queue<pair<int, int> > q;
    q.push(pair<int, int>(start_i, start_j));

    // Keep track of the most extreme values of i and j we've seen, so that we can later draw a
    // bounding box around the blob
    int mini = start_i, maxi = start_i;
    int minj = start_j, maxj = start_j;

    // Count the number of pixels in the blob
    int count = 0;

    // Continue the search until there are no more pixels to explore
    while (!q.empty())
    {
        // Take the first pixel off of the queue
        pair<int, int> p = q.front();
        q.pop();

        int i = p.first;
        int j = p.second;

        // If this pixel is outside of the image, then just ignore it and move to the next iteration
        if (i < 0) continue;
        if (j < 0) continue;
        if (i >= input.rows) continue;
        if (j >= input.cols) continue;

        // If the pixel is "on", then it's part of the blob
        if (input.at<uchar>(i,j) > 0)
        {
            // Increase pixel count
            count++;

            // Zero this pixel in the input image, and label this pixel with the correct index in the output
            input.at<uchar>(i,j) = 0;
            output.at<uchar>(i,j) = index;
          
            // If this pixel is outside our previous bounding box, then update the box
            mini = min(i, mini);
            maxi = max(i, maxi);
            minj = min(j, minj);
            maxj = max(j, maxj);
      
            // Add the adjacent (up, down, left, right) pixels to the queue to explore
            q.push(pair<int, int>(i-1, j));
            q.push(pair<int, int>(i+1, j));
            q.push(pair<int, int>(i, j-1));
            q.push(pair<int, int>(i, j+1));
        }
    }

    // Build a description of the blob we found
    blob b;
    b.pixelCount = count;
    b.index = index;
    b.mini = mini;
    b.maxi = maxi;
    b.minj = minj;
    b.maxj = maxj;

    return b;
}

// Extract blobs from the input image. Descriptions of these blobs will be added to the list found_blobs, and the
// output image will be identical to the input except that each pixel will be labeled with a number that represents
// which blob it is part of.
void labelBlobs(Mat& input, Mat& output, vector<blob>& found_blobs)
{
    // Create a copy of the input image, because extractBlob is a destructive operation (it will actually delete
    // the pixels in the blob as it goes, as a way of book-keeping where it's been before). Since we don't want
    // to destroy the input image, we make a copy and use the copy instead
    Mat temp = input.clone();

    // Iterate over all pixels in the image
    int index = 1;
    for (int i = 0; i < temp.rows; i++)
    {
        for (int j = 0; j < temp.cols; j++)
        {
            // If the pixel is on, then extract the blob around it and add it to the list
            if (temp.at<uchar>(i,j) > 0)
            {
                found_blobs.push_back(extractBlob(temp, output, i, j, index));        

                // I'm lazy, so currently we only support 255 indices.
                // TODO: We should really just change the type of the matrix so that is stores ints instead of chars...
                index++;
                if (index > 255)
                {
                    cout << "Oops, this is embarassing :/" << endl;
                    exit(-1);
                }
            }
        }
    }  
}

// Compute the hsv "distance" between the input image and the specified colour
void getHsvDistance(Mat& image, Scalar colour, int factor = 3)
{
	// Create a new image with colour in HSV colour space
	Mat colorImg(Size(image.cols, image.rows),CV_32FC3);
	colorImg = colour;

	// Calculate the difference between the input image and the colour image
	subtract(image, colorImg, image);

	// Create a scaling matrix for hue scaling
	Mat factorImg(Size(image.cols, image.rows),CV_32FC3);

	// Scale hue because hue is more important that the other channels.
	factorImg = Scalar(factor, 1, 1);
	image = image.mul(factorImg);

}

// Get hsv distance between the given colour and a flag-like red
// The type of the matrix image is expected to be 32F
void getHsvDistanceRed(Mat& image)
{
	getHsvDistance(image, Scalar(170, 200, 200));
}

// Compute the bgr "distance" between two colours
// Note that bgr is just rgb but backwards because for some reason opencv numbers the channels the other
// way round ... *shrugs*
// The type of the matrix image is expected to be 32F
void getBgrDistance(Mat& image, Scalar colour)
{
	// Create a new matrix with specified colour
	Mat colorImg(Size(image.cols, image.rows),CV_32FC3);
	colorImg = colour;

	// subtract the colour image from the original image
	subtract(image, colorImg, image);
}

// Get RGB distance between the given colour and blue
void getBgrDistanceBlue(Mat& image)
{
    return getBgrDistance(image, Scalar(255, 0, 0));
}

// Apply the given function f to each pixel in the input image, and store the value
// at the corresponding location in the output image.
// The function f is expected to take an image of type 32F and perform some manipulations
// on each pixel. (e.g. getBgrDistanceBlue & getHsvDistanceRed)
void imageMap(Mat& input, Mat& output, imgTransform f)
{
	Mat inputF, tmp, mout;

	// Convert image from uchar to 32bit float
	input.convertTo(inputF, CV_32F);

	// Apply the specified function to the image
	f(inputF);

	// Calculate the colour distance using sqrt(b*b + g*g + r*r)/1000

	// Reshape matrix to 'rows*cols' rows by '3 channels' columns
	inputF = inputF.reshape(1, input.rows * input.cols);

	// Square the pixel value of each channel  i.e. R*R, G*G, B*B
	tmp = inputF.mul(inputF);

	// Calculate the sum of the three channels  i.e. ( R*R + G*G + B*B )
	// Reduce to 'rows*cols' rows by'1 channel' column through summation
	reduce(tmp, mout, 1, CV_REDUCE_SUM);
	// Reshape matrix back to original dimension (now it only has 1 channel)
	output = mout.reshape(1, input.rows);

	// calculate the square root of the above sum  i.e. sqrt( R*R + G*G + B*B )
	sqrt(output,output);

	// Scale result by 1/1000  i.e. sqrt( R*R + G*G + B*B ) / 1000
	output = output / 1000;
}

// Apply morphological opening and closing to the input image
void morphologicalFixUp(Mat& input, Mat& output)
{
    //morphological opening (remove small objects from the foreground)
    erode(input, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 

    //morphological closing (fill small holes in the foreground)
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
}


void extractFlagsFromImage(cv::Mat &frame, std::vector<blob> &redFlags, std::vector<blob> &blueFlags)
{

    // Shrink by 2x in both directions
    resize(frame, frame, Size(0, 0), 0.5, 0.5);

    // Gaussian blur to get rid of noise
    Mat blurred;
    GaussianBlur(frame, blurred, Size(15, 15), 5, 5);
    
    // Convert to HSV
    Mat hsv;
    cvtColor(blurred, hsv, CV_BGR2HSV);

    // ***************** Okay, let's find blue flags *****************************

    // Get distance from blue to each pixel
    Mat distBlue;
    imageMap(blurred, distBlue, getBgrDistanceBlue);

	// Threshold the blue image
    Mat threshBlue;
    float maxBlueDist = 0.18;
    inRange(distBlue, Scalar(0), Scalar(maxBlueDist), threshBlue);

    // Do a fixup where we plug small holes and delete small specks of noise
    morphologicalFixUp(threshBlue, threshBlue);

    // Create a dummy image (we won't use this!) to record blob label of each pixel
    Mat dummy = Mat::zeros(threshBlue.rows, threshBlue.cols, CV_8UC1); 

    // Label the blobs
//        vector<blob> blueFlags;
    labelBlobs(threshBlue, dummy, blueFlags);

    // ***************** Done finding blue flags *****************************

    // ***************** Okay, let's find red flags *****************************

    // Get distance from red to each pixel
    Mat distRed;
    imageMap(hsv, distRed, getHsvDistanceRed);

    // Threshold the red image
    Mat threshRed;
    float maxRedDist = 0.12;
    inRange(distRed, Scalar(0), Scalar(maxRedDist), threshRed);

    // Do a fixup where we plug small holes and delete small specks of noise
    morphologicalFixUp(threshRed, threshRed);

    // Label the blobs
//        vector<blob> redFlags;
    labelBlobs(threshRed, dummy, redFlags);

    // ***************** Done finding red flags *****************************

}
