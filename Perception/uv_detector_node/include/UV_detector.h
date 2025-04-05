#ifndef UV_DETECTOR_H
#define UV_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <kalman_filter.h>

using namespace std;
using namespace cv;

/**
 * @brief Class representing a UV detection box
 * 
 * This class stores information about a detected object including:
 * - Unique identifier
 * - Parent relationship
 * - Bounding box coordinates
 */
class UVbox
{
    public:
    // members
    int id;                 ///< Unique identifier for the box
    int toppestParentId;    ///< ID of the highest parent in the hierarchy
    Rect boundingBox;       ///< Bounding box coordinates

    // default constructor
    UVbox();
    // constructor for new line
    UVbox(int segId, int row, int left, int right);
};

/**
 * @brief Class for tracking UV detections over time
 * 
 * This class manages:
 * - Previous and current bounding boxes
 * - Detection history
 * - Kalman filters for tracking
 * - Overlap threshold for tracking validation
 */
class UVtracker
{
    public:
    // members
    vector<Rect> previousBoundingBoxes;     ///< Previous frame's bounding boxes
    vector<Rect> currentBoundingBoxes;      ///< Current frame's bounding boxes
    vector<vector<Point2f>> previousHistory;///< History of previous detections
    vector<vector<Point2f>> currentHistory; ///< History of current detections
    vector<kalman_filter> previousFilters;  ///< Kalman filters for previous states
    vector<kalman_filter> currentFilters;   ///< Kalman filters for current states
    float overlapThreshold;                 ///< Threshold for tracking validation

    // constructor
    UVtracker();

    // read new bounding box information
    void readBoundingBoxes(vector<Rect> currentBoundingBoxes);

    // check tracking status
    void checkStatus();
};

/**
 * @brief Main UV detection class
 * 
 * This class implements the core UV detection algorithm including:
 * - Depth map processing
 * - U-map extraction
 * - Bounding box detection
 * - Bird's eye view transformation
 * - Object tracking
 */
class UVdetector
{
    public:
    // members
    Mat depth;              ///< Input depth map
    Mat depthLowRes;        ///< Low resolution depth map
    Mat uMap;               ///< U-map representation
    int minDistance;        ///< Minimum distance of interest
    int maxDistance;        ///< Maximum distance of interest
    int rowDownsample;      ///< Height ratio (depth/U-map)
    float colScale;         ///< Horizontal scale factor
    float thresholdPoint;   ///< Point of interest threshold
    float thresholdLine;    ///< Line of interest threshold
    int minLineLength;      ///< Minimum line length
    bool showBoundingBoxU;  ///< Flag to show U-map bounding boxes
    vector<Rect> boundingBoxU;  ///< Bounding boxes on U-map
    vector<Rect> boundingBoxB;  ///< Bounding boxes on bird's view
    float fx;               ///< Focal length x
    float fy;               ///< Focal length y
    float px;               ///< Principal point x
    float py;               ///< Principal point y
    Mat birdView;           ///< Bird's eye view map
    UVtracker tracker;      ///< Object tracker

    // constructor
    UVdetector();

    // read data
    void readData(Mat depth);

    // extract U map
    void extractUMap();

    // extract bounding box
    void extractBoundingBoxes();

    // extract bird's view map
    void extractBirdView();

    // detect
    void detect();

    // track the object
    void track();

    // output detection 
    void output();

    // display depth
    void displayDepth();

    // display U map
    void displayUMap();

    // add tracking result to bird's view map
    void addTrackingResult();

    // display bird's view map
    void displayBirdView();
};

/**
 * @brief Merges two UV boxes
 * @param father Parent UV box
 * @param son Child UV box
 * @return Merged UV box
 */
UVbox mergeTwoUVbox(UVbox father, UVbox son);

#endif