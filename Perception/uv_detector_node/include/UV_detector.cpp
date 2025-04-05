/**
 * @file UV_detector.cpp
 * @brief Implementation of UV-based object detection and tracking
 * 
 * This file contains the implementation of UV-based object detection and tracking
 * using depth data and Kalman filtering for state estimation.
 */

// Tracking Parameters
#define OVERLAP_THRESHOLD 0.51    // Minimum overlap ratio between consecutive frames to consider objects as the same (higher = stricter tracking)
#define TRACKING_FREQUENCY 30     // Hz - Update rate of the Kalman filter (higher = smoother but more computation)
#define PROCESS_NOISE 0.4         // Process noise covariance for Kalman filter (higher = more responsive to changes)
#define PROCESS_NOISE_SCALE 0.0   // Additional scaling factor for process noise (fine-tunes filter responsiveness)
#define MEASUREMENT_NOISE 0.3     // Measurement noise covariance for Kalman filter (higher = less trust in measurements)
#define MEASUREMENT_NOISE_SCALE 0.99  // Additional scaling factor for measurement noise (fine-tunes measurement trust)

// Detection Parameters
#define ROW_DOWNSAMPLE 4          // Factor to downsample rows in depth image (higher = less computation but lower vertical resolution)
#define COL_SCALE 0.5             // Scale factor for columns in depth image (higher = more detail but more computation)
#define MIN_DISTANCE 10           // mm - Minimum valid depth value (objects closer are ignored)
#define MAX_DISTANCE 5000         // mm - Maximum valid depth value (objects further are ignored)
#define THRESHOLD_POINT 2         // Minimum value for a point to be considered valid in U-map (higher = less noise but may miss weak detections)
#define THRESHOLD_LINE 2          // Threshold for line detection in U-map (higher = requires stronger evidence)
#define MIN_LENGTH_LINE 8         // Minimum length for a valid line segment (higher = fewer false positives but may miss small objects)
#define SHOW_BOUNDING_BOX_U true  // Flag to display bounding boxes in U-map visualization (useful for debugging)

// Camera Parameters
#define FOCAL_LENGTH_X 383.91     // Camera focal length in x direction (pixels) - affects field of view and object size
#define FOCAL_LENGTH_Y 383.91     // Camera focal length in y direction (pixels) - affects field of view and object size
#define PRINCIPAL_POINT_X 318.27  // Principal point x coordinate (pixels) - center of image in x direction
#define PRINCIPAL_POINT_Y 242.18  // Principal point y coordinate (pixels) - center of image in y direction

// Visualization Parameters
#define BIRD_VIEW_WIDTH 1000      // Width of the bird's eye view visualization (affects resolution)
#define BIRD_VIEW_HEIGHT 500      // Height of the bird's eye view visualization (affects resolution)
#define BIRD_VIEW_SCALE 0.5       // Scale factor for displaying the bird's eye view (higher = more detail but needs more screen space)

// Ground Plane Parameters
#define GROUND_HEIGHT_MIN 0.2     // Minimum height (m) above ground to consider a point as an obstacle (lower = detects shorter objects but more noise)
#define GROUND_HEIGHT_MAX 1.7     // Maximum height (m) to consider for obstacles (higher = detects taller objects but may include overhead structures)
#define GROUND_FIT_THRESHOLD 0.02 // RANSAC threshold (m) for ground plane fitting (lower = stricter plane fit but may miss uneven ground)
#define REMOVE_GROUND true        // Enable/disable ground plane removal (true = remove ground points, false = keep all points)

#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <UV_detector.h>
#include <kalman_filter.h>

using namespace std;
using namespace cv;

/**
 * @brief UVbox class implementation
 * 
 * Represents a bounding box with tracking information for UV-based detection.
 */
UVbox::UVbox()
{
    this->id = 0;
    this->toppestParentId = 0;
    // Initialize bounding box with zero size at origin
    this->boundingBox = Rect(Point2f(0, 0), Point2f(0, 0));
}

/**
 * @brief Constructor for UVbox with specific parameters
 * @param segId - Segmentation ID: unique identifier for each detected segment/object
 * @param row - Row position in the image
 * @param left - Left boundary of the bounding box
 * @param right - Right boundary of the bounding box
 */
UVbox::UVbox(int segId, int row, int left, int right)
{
    this->id = segId;
    this->toppestParentId = segId;
    // Create a bounding box with the given coordinates
    this->boundingBox = Rect(Point2f(left, row), Point2f(right, row));
}

/**
 * @brief Merges two UVboxes into one
 * @param father - The parent UVbox to merge into
 * @param son - The child UVbox to merge
 * @return The merged UVbox
 */
UVbox mergeTwoUVbox(UVbox father, UVbox son)
{
    // Merge the bounding boxes by finding the minimum and maximum coordinates
    // tl() returns the top-left point of the rectangle
    // br() returns the bottom-right point of the rectangle
    int top = min(father.boundingBox.tl().y, son.boundingBox.tl().y);
    int left = min(father.boundingBox.tl().x, son.boundingBox.tl().x);
    int bottom = max(father.boundingBox.br().y, son.boundingBox.br().y);
    int right = max(father.boundingBox.br().x, son.boundingBox.br().x);
    // Create a new rectangle that encompasses both boxes
    father.boundingBox = Rect(Point2f(left, top), Point2f(right, bottom));
    return father;
}

/**
 * @brief UVtracker class implementation
 * 
 * Handles object tracking using Kalman filters and bounding box matching.
 */
UVtracker::UVtracker()
{
    // Threshold for determining if two bounding boxes overlap enough to be considered the same object
    this->overlapThreshold = OVERLAP_THRESHOLD;
}

/**
 * @brief Updates the tracker with new bounding boxes
 * @param currentBoundingBoxes - Vector of current frame's bounding boxes
 * 
 * This method maintains three types of data:
 * 1. Measurement history for trajectory tracking
 * 2. Kalman filters for state estimation
 * 3. Bounding box data for overlap detection
 */
void UVtracker::readBoundingBoxes(vector<Rect> currentBoundingBoxes)
{
    // Update measurement history
    this->previousHistory = this->currentHistory;
    this->currentHistory.clear();
    this->currentHistory.resize(currentBoundingBoxes.size());
    
    // Update Kalman filters
    this->previousFilters = this->currentFilters;
    this->currentFilters.clear();
    this->currentFilters.resize(currentBoundingBoxes.size());
    
    // Update bounding boxes
    this->previousBoundingBoxes = this->currentBoundingBoxes;
    this->currentBoundingBoxes = currentBoundingBoxes;
}

/**
 * @brief Checks the status of tracked objects and updates their states
 * 
 * For each current bounding box:
 * 1. Checks if it overlaps with any previous bounding box
 * 2. If overlap found, updates the Kalman filter with new measurement
 * 3. If no overlap, initializes a new Kalman filter for tracking
 */
void UVtracker::checkStatus()
{
    for(int currentId = 0; currentId < this->currentBoundingBoxes.size(); currentId++)
    {
        bool tracked = false;
        
        // Calculate center point once
        float centerX = this->currentBoundingBoxes[currentId].x + 0.5 * this->currentBoundingBoxes[currentId].width;
        float centerY = this->currentBoundingBoxes[currentId].y + 0.5 * this->currentBoundingBoxes[currentId].height;
        
        for(int previousId = 0; previousId < this->previousBoundingBoxes.size(); previousId++)
        {
            
            
            // Calculate overlap between current and previous bounding boxes
            Rect overlap = this->currentBoundingBoxes[currentId] & this->previousBoundingBoxes[previousId];
            float overlapRatio = overlap.area() / float(this->currentBoundingBoxes[currentId].area());
            if(overlapRatio >= this->overlapThreshold)
            {
                tracked = true;
                
                // Add current detection to history
                this->currentHistory[currentId] = this->previousHistory[previousId];
                this->currentHistory[currentId].push_back(Point2f(centerX, centerY));
                
                // Add measurement to previous filter
                this->currentFilters[currentId] = this->previousFilters[previousId];
                MatrixXd measurement(4,1);
                measurement << centerX,
                             centerY,
                             this->currentBoundingBoxes[currentId].width,
                             this->currentBoundingBoxes[currentId].height;
                MatrixXd controlInput(1,1);
                controlInput << 0;
                
                // Run the filter
                this->currentFilters[currentId].estimate(measurement, controlInput);
                break;
            }
        }
        if(!tracked)
        {
            // Add current detection to history
            this->currentHistory[currentId].push_back(Point2f(centerX, centerY));
            
            // Initialize filter
            int frequency = TRACKING_FREQUENCY; // Hz
            double timeStep = 1.0 / frequency; // s
            
            // Model for center filter
            double processNoise = PROCESS_NOISE;
            double processNoiseScale = PROCESS_NOISE_SCALE;
            double measurementNoise = MEASUREMENT_NOISE;
            double measurementNoiseScale = MEASUREMENT_NOISE_SCALE;
            
            MatrixXd stateMatrix(6, 6);
            stateMatrix << 1, 0, timeStep, 0, 0, 0, 
                          0, 1, 0, timeStep, 0, 0,
                          0, 0, 1, 0, 0, 0,
                          0, 0, 0, 1, 0, 0,
                          0, 0, 0, 0, 1, 0,
                          0, 0, 0, 0, 0, 1;
            
            MatrixXd inputMatrix(6, 1);
            inputMatrix << 0, 0, 0, 0, 0, 0;
            
            MatrixXd observationMatrix(4, 6);
            observationMatrix << 1, 0, 0, 0, 0, 0,
                               0, 1, 0, 0, 0, 0,
                               0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 1;
            
            MatrixXd uncertainty = MatrixXd::Identity(6, 6) * measurementNoise;
            uncertainty(4,4) = measurementNoiseScale;
            uncertainty(5,5) = measurementNoiseScale;
            
            MatrixXd processNoiseMatrix = MatrixXd::Identity(6, 6) * processNoise;
            processNoiseMatrix(4,4) = processNoiseScale;
            processNoiseMatrix(5,5) = processNoiseScale;
            
            MatrixXd measurementNoiseMatrix = MatrixXd::Identity(4, 4) * measurementNoise;

            // Filter initialization
            MatrixXd states(6,1);
            states << centerX,
                     centerY,
                     0, 0,
                     this->currentBoundingBoxes[currentId].width,
                     this->currentBoundingBoxes[currentId].height;
            
            this->currentFilters[currentId].setup(states,
                                               stateMatrix,
                                               inputMatrix,
                                               observationMatrix,
                                               uncertainty,
                                               processNoiseMatrix,
                                               measurementNoiseMatrix);
        }
    }
}

/**
 * @brief UVdetector class implementation
 * 
 * Main class for UV-based object detection using depth data.
 */
UVdetector::UVdetector()
{
    // Image processing parameters
    this->rowDownsample = ROW_DOWNSAMPLE;
    this->colScale = COL_SCALE;
    this->minDistance = MIN_DISTANCE;
    this->maxDistance = MAX_DISTANCE;
    this->thresholdPoint = THRESHOLD_POINT;
    this->thresholdLine = THRESHOLD_LINE;
    this->minLengthLine = MIN_LENGTH_LINE;
    this->showBoundingBoxU = SHOW_BOUNDING_BOX_U;
    
    // Camera calibration parameters
    this->focalLengthX = FOCAL_LENGTH_X;
    this->focalLengthY = FOCAL_LENGTH_Y;
    this->principalPointX = PRINCIPAL_POINT_X;
    this->principalPointY = PRINCIPAL_POINT_Y;

    // Ground plane parameters
    this->groundHeightMin = GROUND_HEIGHT_MIN;
    this->groundHeightMax = GROUND_HEIGHT_MAX;
    this->groundFitThreshold = GROUND_FIT_THRESHOLD;
    this->removeGround = REMOVE_GROUND;
}

/**
 * @brief Reads depth data from the camera
 * @param depth - Input depth image
 */
void UVdetector::readData(Mat depth)
{
    this->depth = depth;
}

/**
 * @brief Fits ground plane using RANSAC on depth data
 * 
 * Uses organized point cloud data to fit a plane to ground points.
 * Points are converted from depth to 3D coordinates using camera parameters.
 */
void UVdetector::fitGroundPlane()
{
    if (!this->removeGround) return;

    // Initialize ground mask
    this->groundMask = Mat::zeros(this->depth.rows, this->depth.cols, CV_8UC1);
    
    // Convert depth to 3D points
    vector<Point3f> points;
    vector<Point2i> pixels;  // Keep track of pixel coordinates
    
    for(int row = 0; row < this->depth.rows; row++) {
        for(int col = 0; col < this->depth.cols; col++) {
            float d = this->depth.at<unsigned short>(row, col);
            if(d > this->minDistance && d < this->maxDistance) {
                // Convert depth to 3D point
                float x = (col - this->principalPointX) * d / this->focalLengthX;
                float y = (row - this->principalPointY) * d / this->focalLengthY;
                float z = d;
                points.push_back(Point3f(x, y, z));
                pixels.push_back(Point2i(col, row));
            }
        }
    }

    if(points.size() < 3) return;  // Need at least 3 points for plane fitting

    // RANSAC parameters
    int iterations = 100;
    float bestScore = 0;
    Vec4f bestPlane;
    
    // RANSAC iterations
    for(int iter = 0; iter < iterations; iter++) {
        // Randomly select 3 points
        vector<Point3f> sample;
        vector<int> indices;
        for(int i = 0; i < 3; i++) {
            int idx;
            do {
                idx = rand() % points.size();
            } while(find(indices.begin(), indices.end(), idx) != indices.end());
            indices.push_back(idx);
            sample.push_back(points[idx]);
        }
        
        // Fit plane to three points
        Point3f v1 = sample[1] - sample[0];
        Point3f v2 = sample[2] - sample[0];
        Point3f normal = v1.cross(v2);
        float d = -normal.dot(sample[0]);
        Vec4f plane(normal.x, normal.y, normal.z, d);
        
        // Count inliers
        float score = 0;
        for(const Point3f& pt : points) {
            float dist = abs(plane[0]*pt.x + plane[1]*pt.y + plane[2]*pt.z + plane[3]) / 
                        sqrt(plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2]);
            if(dist < this->groundFitThreshold) {
                score++;
            }
        }
        
        // Update best plane
        if(score > bestScore) {
            bestScore = score;
            bestPlane = plane;
        }
    }
    
    this->groundPlane = bestPlane;
    
    // Create ground mask
    for(size_t i = 0; i < points.size(); i++) {
        const Point3f& pt = points[i];
        float dist = abs(bestPlane[0]*pt.x + bestPlane[1]*pt.y + bestPlane[2]*pt.z + bestPlane[3]) / 
                    sqrt(bestPlane[0]*bestPlane[0] + bestPlane[1]*bestPlane[1] + bestPlane[2]*bestPlane[2]);
        
        if(dist < this->groundFitThreshold) {
            this->groundMask.at<uchar>(pixels[i].y, pixels[i].x) = 255;
        }
    }
}

/**
 * @brief Removes ground points from depth image
 * 
 * Uses the fitted ground plane to remove points that are likely ground.
 * Also applies height constraints from the paper.
 */
void UVdetector::removeGroundPoints()
{
    if (!this->removeGround) return;
    
    Mat depthNoGround = this->depth.clone();
    
    for(int row = 0; row < this->depth.rows; row++) {
        for(int col = 0; col < this->depth.cols; col++) {
            if(this->groundMask.at<uchar>(row, col) > 0) {
                // Point is part of ground plane
                depthNoGround.at<unsigned short>(row, col) = 0;
                continue;
            }
            
            float d = this->depth.at<unsigned short>(row, col);
            if(d > this->minDistance && d < this->maxDistance) {
                // Convert to 3D point
                float x = (col - this->principalPointX) * d / this->focalLengthX;
                float y = (row - this->principalPointY) * d / this->focalLengthY;
                float z = d;
                
                // Calculate height above ground plane
                float height = abs(this->groundPlane[0]*x + this->groundPlane[1]*y + 
                                 this->groundPlane[2]*z + this->groundPlane[3]) / 
                             sqrt(this->groundPlane[0]*this->groundPlane[0] + 
                                  this->groundPlane[1]*this->groundPlane[1] + 
                                  this->groundPlane[2]*this->groundPlane[2]);
                
                // Apply height constraints
                if(height < this->groundHeightMin || height > this->groundHeightMax) {
                    depthNoGround.at<unsigned short>(row, col) = 0;
                }
            }
        }
    }
    
    this->depth = depthNoGround;
}

/**
 * @brief Extracts the U-map from depth data
 * 
 * The U-map is a 2D representation where:
 * - Rows represent depth bins
 * - Columns represent image columns
 * - Values represent the number of points in each bin
 */
void UVdetector::extractUMap()
{
    // Rescale depth map
    Mat depthRescale;
    resize(this->depth, depthRescale, Size(), this->colScale, 1);
    Mat depthLowResTemp = Mat::zeros(depthRescale.rows, depthRescale.cols, CV_8UC1);

    // Construct the mask
    Rect maskDepth;
    uint8_t histSize = this->depth.rows / this->rowDownsample;
    uint8_t binWidth = ceil((this->maxDistance - this->minDistance) / float(histSize));
    
    // Initialize U map
    this->uMap = Mat::zeros(histSize, depthRescale.cols, CV_8UC1);
    
    for(int col = 0; col < depthRescale.cols; col++)
    {
        for(int row = 0; row < depthRescale.rows; row++)
        {
            if(depthRescale.at<unsigned short>(row, col) > this->minDistance && 
               depthRescale.at<unsigned short>(row, col) < this->maxDistance)
            {
                uint8_t binIndex = (depthRescale.at<unsigned short>(row, col) - this->minDistance) / binWidth;
                depthLowResTemp.at<uchar>(row, col) = binIndex;
                if(this->uMap.at<uchar>(binIndex, col) < 255)
                {
                    this->uMap.at<uchar>(binIndex, col)++;
                }
            }
        }
    }
    this->depthLowRes = depthLowResTemp;

    // Smooth the U map
    GaussianBlur(this->uMap, this->uMap, Size(5,9), 3, 10);
}

/**
 * @brief Extracts bounding boxes from the U-map
 * 
 * This method:
 * 1. Scans the U-map row by row
 * 2. Identifies continuous segments of interest
 * 3. Merges segments vertically to form complete objects
 * 4. Filters out small or invalid detections
 */
void UVdetector::extractBoundingBoxes()
{
    // Initialize a mask
    vector<vector<int>> mask(this->uMap.rows, vector<int>(this->uMap.cols, 0));
    
    // Initialize parameters
    int uMin = this->thresholdPoint * this->rowDownsample;
    int sumLine = 0;
    int maxLine = 0;
    int lengthLine = 0;
    int segmentId = 0;
    vector<UVbox> uvBoxes;
    
    for(int row = 0; row < this->uMap.rows; row++)
    {
        for(int col = 0; col < this->uMap.cols; col++)
        {
            // Check if point is of interest
            if(this->uMap.at<uchar>(row,col) >= uMin)
            {
                // Update current line info
                lengthLine++;
                sumLine += this->uMap.at<uchar>(row,col);
                maxLine = max(this->uMap.at<uchar>(row,col), maxLine);
            }
            
            // Check if not of interest or end of row
            if(this->uMap.at<uchar>(row,col) < uMin || col == this->uMap.cols - 1)
            {
                // Handle end of row
                col = (col == this->uMap.cols - 1) ? col + 1 : col;
                
                // Check if good line candidate (length and sum)
                if(lengthLine > this->minLengthLine && sumLine > this->thresholdLine * maxLine)
                {
                    segmentId++;
                    uvBoxes.push_back(UVbox(segmentId, row, col - lengthLine, col - 1));
                    
                    // Update mask with segmentation id
                    for(int c = col - lengthLine; c < col - 1; c++)
                    {
                        mask[row][c] = segmentId;
                    }
                    
                    // Merge neighbor segmentation if not first row
                    if(row != 0)
                    {
                        for(int c = col - lengthLine; c < col - 1; c++)
                        {
                            if(mask[row - 1][c] != 0)
                            {
                                if(uvBoxes[mask[row - 1][c] - 1].toppestParentId < uvBoxes.back().toppestParentId)
                                {
                                    uvBoxes.back().toppestParentId = uvBoxes[mask[row - 1][c] - 1].toppestParentId;
                                }
                                else
                                {
                                    int temp = uvBoxes[mask[row - 1][c] - 1].toppestParentId;
                                    for(int b = 0; b < uvBoxes.size(); b++)
                                    {
                                        uvBoxes[b].toppestParentId = (uvBoxes[b].toppestParentId == temp) ? 
                                                                    uvBoxes.back().toppestParentId : 
                                                                    uvBoxes[b].toppestParentId;
                                    }
                                }
                            }
                        }
                    }
                }
                // Reset line parameters
                lengthLine = 0;
                sumLine = 0;
                maxLine = 0;
            }
        }
    }
    
    // Store the detected boxes
    this->boundingBoxes = uvBoxes;
}

/**
 * @brief Main detection pipeline
 * 
 * Executes the complete detection process:
 * 1. Fits and removes ground plane
 * 2. Extracts U-map from depth data
 * 3. Extracts bounding boxes from U-map
 * 4. Converts to bird's eye view
 */
void UVdetector::detect()
{
    // Fit and remove ground plane
    this->fitGroundPlane();
    this->removeGroundPoints();

    // Extract U map from depth
    this->extractUMap();

    // Extract bounding box from U map
    this->extractBoundingBoxes();

    // Extract bounding box
    this->extractBirdView();
}

/**
 * @brief Displays the depth image
 */
void UVdetector::displayDepth()
{
    cvtColor(this->depth, this->depth, CV_GRAY2RGB);
    imshow("Depth", this->depth);
    waitKey(1);
}

/**
 * @brief Displays the U-map with bounding boxes
 */
void UVdetector::displayUMap()
{
    // Visualize with bounding box
    if(this->showBoundingBoxU)
    {
        this->uMap = this->uMap * 10;
        cvtColor(this->uMap, this->uMap, CV_GRAY2RGB);
        for(int b = 0; b < this->boundingBoxes.size(); b++)
        {
            Rect finalBoundingBox = Rect(this->boundingBoxes[b].boundingBox.tl(),
                                       Size(this->boundingBoxes[b].boundingBox.width, 
                                           2 * this->boundingBoxes[b].boundingBox.height));
            rectangle(this->uMap, finalBoundingBox, Scalar(0, 0, 255), 1, 8, 0);
            circle(this->uMap, 
                  Point2f(this->boundingBoxes[b].boundingBox.tl().x + 0.5 * this->boundingBoxes[b].boundingBox.width, 
                         this->boundingBoxes[b].boundingBox.br().y), 
                  2, Scalar(0, 0, 255), 5, 8, 0);
        }
    }
    
    imshow("U map", this->uMap);
    waitKey(1);
}

/**
 * @brief Converts detected boxes to bird's eye view
 * 
 * This method:
 * 1. Projects 2D image coordinates to 3D world coordinates
 * 2. Creates a top-down view of the detected objects
 */
void UVdetector::extractBirdView()
{
    // Extract bounding boxes in bird's view
    uint8_t histSize = this->depth.rows / this->rowDownsample;
    uint8_t binWidth = ceil((this->maxDistance - this->minDistance) / float(histSize));
    this->boundingBoxesB.clear();
    this->boundingBoxesB.resize(this->boundingBoxes.size());

    for(int b = 0; b < this->boundingBoxes.size(); b++)
    {
        // U map bounding box -> Bird's view bounding box conversion
        float bbDepth = this->boundingBoxes[b].boundingBox.br().y * binWidth / 10;
        float bbWidth = bbDepth * this->boundingBoxes[b].boundingBox.width / this->focalLengthX;
        float bbHeight = this->boundingBoxes[b].boundingBox.height * binWidth / 10;
        float bbX = bbDepth * (this->boundingBoxes[b].boundingBox.tl().x / this->colScale - this->principalPointX) / this->focalLengthX;
        float bbY = bbDepth - 0.5 * bbHeight;
        this->boundingBoxesB[b] = Rect(bbX, bbY, bbWidth, bbHeight);
    }

    // Initialize the bird's view
    this->birdView = Mat::zeros(BIRD_VIEW_HEIGHT, BIRD_VIEW_WIDTH, CV_8UC1);
    cvtColor(this->birdView, this->birdView, CV_GRAY2RGB);
}

/**
 * @brief Displays the bird's eye view with detected objects
 */
void UVdetector::displayBirdView()
{
    // Center point
    Point2f center = Point2f(this->birdView.cols / 2, this->birdView.rows);
    Point2f leftEndToCenter = Point2f(this->birdView.rows * (0 - this->principalPointX) / this->focalLengthX, -this->birdView.rows);
    Point2f rightEndToCenter = Point2f(this->birdView.rows * (this->depth.cols - this->principalPointX) / this->focalLengthX, -this->birdView.rows);

    // Draw the two side lines
    line(this->birdView, center, center + leftEndToCenter, Scalar(0, 255, 0), 3, 8, 0);
    line(this->birdView, center, center + rightEndToCenter, Scalar(0, 255, 0), 3, 8, 0);

    for(int b = 0; b < this->boundingBoxes.size(); b++)
    {
        // Change coordinates
        Rect finalBoundingBox = this->boundingBoxesB[b];
        finalBoundingBox.y = center.y - finalBoundingBox.y - finalBoundingBox.height;
        finalBoundingBox.x = finalBoundingBox.x + center.x;
        
        // Draw center
        Point2f bbCenter = Point2f(finalBoundingBox.x + 0.5 * finalBoundingBox.width, 
                                  finalBoundingBox.y + 0.5 * finalBoundingBox.height);
        rectangle(this->birdView, finalBoundingBox, Scalar(0, 0, 255), 3, 8, 0);
        circle(this->birdView, bbCenter, 3, Scalar(0, 0, 255), 5, 8, 0);
    }

    // Show
    resize(this->birdView, this->birdView, Size(), BIRD_VIEW_SCALE, BIRD_VIEW_SCALE);
    imshow("Bird's View", this->birdView);
    waitKey(1);
}

/**
 * @brief Adds tracking results to the bird's eye view
 * 
 * This method:
 * 1. Projects tracked object positions to bird's eye view
 * 2. Draws estimated positions and velocities
 * 3. Shows object trajectories
 */
void UVdetector::addTrackingResult()
{
    Point2f center = Point2f(this->birdView.cols / 2, this->birdView.rows);
    for(int b = 0; b < this->tracker.currentBoundingBoxes.size(); b++)
    {
        // Change coordinates
        Point2f estimatedCenter = Point2f(this->tracker.currentFilters[b].output(0), 
                                        this->tracker.currentFilters[b].output(1));
        estimatedCenter.y = center.y - estimatedCenter.y;
        estimatedCenter.x = estimatedCenter.x + center.x;
        
        // Draw center
        circle(this->birdView, estimatedCenter, 5, Scalar(0, 255, 0), 5, 8, 0);
        
        // Draw bounding box
        Point2f bbSize = Point2f(this->tracker.currentFilters[b].output(4), 
                                this->tracker.currentFilters[b].output(5));
        rectangle(this->birdView, 
                 Rect(estimatedCenter - 0.5 * bbSize, estimatedCenter + 0.5 * bbSize), 
                 Scalar(0, 255, 0), 3, 8, 0);
        
        // Draw velocity
        Point2f velocity = Point2f(this->tracker.currentFilters[b].output(2), 
                                 this->tracker.currentFilters[b].output(3));
        velocity.y = -velocity.y;
        line(this->birdView, estimatedCenter, estimatedCenter + velocity, 
             Scalar(255, 255, 255), 3, 8, 0);
        
        for(int h = 1; h < this->tracker.currentHistory[b].size(); h++)
        {
            // Trajectory
            Point2f start = this->tracker.currentHistory[b][h - 1];
            start.y = center.y - start.y;
            start.x = start.x + center.x;
            Point2f end = this->tracker.currentHistory[b][h];
            end.y = center.y - end.y;
            end.x = end.x + center.x;
            line(this->birdView, start, end, Scalar(0, 0, 255), 3, 8, 0);
        }
    }
}

/**
 * @brief Main tracking pipeline
 * 
 * Executes the complete tracking process:
 * 1. Updates tracker with new bounding boxes
 * 2. Checks object status and updates filters
 * 3. Adds tracking results to visualization
 */
void UVdetector::track()
{
    this->tracker.readBoundingBoxes(this->boundingBoxesB);
    this->tracker.checkStatus();
    this->addTrackingResult();
}

