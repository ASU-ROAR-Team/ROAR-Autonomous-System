/**
 * @file UV_detector.cpp
 * @brief Implementation of UV-based object detection and tracking
 * 
 * This file contains the implementation of UV-based object detection and tracking
 * using depth data and Kalman filtering for state estimation.
 */

/*I am trying to use the zed2i camera instead of the intel realsense one, but their default height x width px are different, is this code specific to a certain height x width or a certain aspect ratio, if so show me where */

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
#define REMOVE_GROUND false        // Enable/disable ground plane removal (true = remove ground points, false = keep all points)


#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <UV_detector.h>
#include <kalman_filter.h>

using namespace std;
using namespace cv;

// UVbox

UVbox::UVbox()
{
    this->id = 0;
    this->toppest_parent_id = 0;
    this->bb = Rect(Point2f(0, 0), Point2f(0, 0));
}

UVbox::UVbox(int seg_id, int row, int left, int right)
{
    this->id = seg_id;
    this->toppest_parent_id = seg_id;
    this->bb = Rect(Point2f(left, row), Point2f(right, row));
}

UVbox merge_two_UVbox(UVbox father, UVbox son)
{
    // merge the bounding box
    int top =       (father.bb.tl().y < son.bb.tl().y)?father.bb.tl().y:son.bb.tl().y;
    int left =      (father.bb.tl().x < son.bb.tl().x)?father.bb.tl().x:son.bb.tl().x;
    int bottom =    (father.bb.br().y > son.bb.br().y)?father.bb.br().y:son.bb.br().y;
    int right =     (father.bb.br().x > son.bb.br().x)?father.bb.br().x:son.bb.br().x;
    father.bb = Rect(Point2f(left, top), Point2f(right, bottom));
    return father;
}

// UVtracker

UVtracker::UVtracker()
{
    this->overlap_threshold = OVERLAP_THRESHOLD;
}

void UVtracker::read_bb(vector<Rect> now_bb)
{
    // measurement history
    this->pre_history = this->now_history;
    this->now_history.clear();
    this->now_history.resize(now_bb.size());
    // kalman filters
    this->pre_filter = this->now_filter;
    this->now_filter.clear();
    this->now_filter.resize(now_bb.size());
    // bounding box
    this->pre_bb = this->now_bb;
    this->now_bb = now_bb;
}

void UVtracker::check_status()
{
    for(int now_id = 0; now_id < this->now_bb.size(); now_id++)
    {
        bool tracked = false;
        for(int pre_id = 0; pre_id < this->pre_bb.size(); pre_id++)
        {
            Rect overlap = this->now_bb[now_id] & this->pre_bb[pre_id];
            if(min(overlap.area() / float(this->now_bb[now_id].area()), overlap.area() / float(this->now_bb[now_id].area())) >= this->overlap_threshold)
            {
                tracked = true;
                // add current detection to history
                this->now_history[now_id] = this->pre_history[pre_id];
                this->now_history[now_id].push_back(Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
                // add measurement to previous filter
                this->now_filter[now_id] = this->pre_filter[pre_id];
                MatrixXd z(4,1); // measurement
                z << this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height, this->now_bb[now_id].width, this->now_bb[now_id].height;
                MatrixXd u(1,1); // input
                u << 0;
                // run the filter 
                this->now_filter[now_id].estimate(z, u);
                break;
            }
        }
        if(!tracked)
        {
            // add current detection to history
            this->now_history[now_id].push_back(Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
            // initialize filter
            int f = TRACKING_FREQUENCY; // Hz
            double ts = 1.0 / f; // s
            
            // model for center filter
            double e_p = PROCESS_NOISE;
            double e_ps = PROCESS_NOISE_SCALE;
            double e_m = MEASUREMENT_NOISE;
            double e_ms = MEASUREMENT_NOISE_SCALE;
            MatrixXd A(6, 6);
            A <<    1, 0, ts, 0, 0, 0, 
                    0, 1, 0, ts, 0, 0,
                    0, 0, 1, 0,  0, 0,
                    0, 0, 0, 1,  0, 0,
                    0, 0, 0, 0,  1, 0,
                    0, 0, 0, 0,  0, 1; 
            MatrixXd B(6, 1);
            B <<    0, 0, 0, 0, 0, 0;
            MatrixXd H(4, 6);
            H <<    1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 1;
            MatrixXd P = MatrixXd::Identity(6, 6) * e_m;
            P(4,4) = e_ms; P(5,5) = e_ms;
            MatrixXd Q = MatrixXd::Identity(6, 6) * e_p;
            Q(4,4) = e_ps; Q(5,5) = e_ps;
            MatrixXd R = MatrixXd::Identity(4, 4) * e_m;

            // filter initialization
            MatrixXd states(6,1);
            states << this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height, 0, 0, this->now_bb[now_id].width, this->now_bb[now_id].height;
            kalman_filter my_filter;
            
            this->now_filter[now_id].setup(states,
                            A,
                            B,
                            H,
                            P,
                            Q,
                            R);
        }
    }
}

// UVdetector

UVdetector::UVdetector()
{
    // Image processing parameters
    this->row_downsample = ROW_DOWNSAMPLE;
    this->col_scale = COL_SCALE;
    this->min_dist = MIN_DISTANCE;
    this->max_dist = MAX_DISTANCE;
    this->threshold_point = THRESHOLD_POINT;
    this->threshold_line = THRESHOLD_LINE;
    this->min_length_line = MIN_LENGTH_LINE;
    this->show_bounding_box_U = SHOW_BOUNDING_BOX_U;

    // Camera calibration parameters
    this->fx = FOCAL_LENGTH_X;
    this->fy = FOCAL_LENGTH_Y;
    this->px = PRINCIPAL_POINT_X;
    this->py = PRINCIPAL_POINT_Y;

    // Ground plane parameters
    this->groundHeightMin = GROUND_HEIGHT_MIN;
    this->groundHeightMax = GROUND_HEIGHT_MAX;
    this->groundFitThreshold = GROUND_FIT_THRESHOLD;
    this->removeGround = REMOVE_GROUND;
}

void UVdetector::readdata(Mat depth)
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
             if(d > this->min_dist && d < this->max_dist) {
                 // Convert depth to 3D point
                 float x = (col - this->px) * d / this->fx;
                 float y = (row - this->py) * d / this->fy;
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
             if(d > this->min_dist && d < this->max_dist) {
                 // Convert to 3D point
                 float x = (col - this->px) * d / this->fx;
                 float y = (row - this->py) * d / this->fy;
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

 
void UVdetector::extract_U_map()
{
    // rescale depth map
    Mat depth_rescale;
    resize(this->depth, depth_rescale, Size(),this->col_scale , 1);
    Mat depth_low_res_temp = Mat::zeros(depth_rescale.rows, depth_rescale.cols, CV_8UC1);

    // construct the mask
    Rect mask_depth;
    uint8_t histSize = this->depth.rows / this->row_downsample;
    uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
    // initialize U map
    this->U_map = Mat::zeros(histSize, depth_rescale.cols, CV_8UC1);
    for(int col = 0; col < depth_rescale.cols; col++)
    {
        for(int row = 0; row < depth_rescale.rows; row++)
        {
            if(depth_rescale.at<unsigned short>(row, col) > this->min_dist && depth_rescale.at<unsigned short>(row, col) < this->max_dist)
            {
                uint8_t bin_index = (depth_rescale.at<unsigned short>(row, col) - this->min_dist) / bin_width;
                depth_low_res_temp.at<uchar>(row, col) = bin_index;
                if(this->U_map.at<uchar>(bin_index, col) < 255)
                {
                    this->U_map.at<uchar>(bin_index, col) ++;
                }
            }
        }
    }
    this->depth_low_res = depth_low_res_temp;

    // smooth the U map
    GaussianBlur(this->U_map, this->U_map, Size(5,9), 3, 10);
}

void UVdetector::extract_bb()
{
    // initialize a mask
    vector<vector<int> > mask(this->U_map.rows, vector<int>(this->U_map.cols, 0));
    // initialize parameters
    int u_min = this->threshold_point * this->row_downsample;
    int sum_line = 0;
    int max_line = 0;
    int length_line = 0;
    int seg_id = 0;
    vector<UVbox> UVboxes;
    for(int row = 0; row < this->U_map.rows; row++)
    {
        for(int col = 0; col < this->U_map.cols; col++)
        {
            // is a point of interest
            if(this->U_map.at<uchar>(row,col) >= u_min)
            {
                // update current line info
                length_line++;
                sum_line += this->U_map.at<uchar>(row,col);
                max_line = (this->U_map.at<uchar>(row,col) > max_line)?this->U_map.at<uchar>(row,col):max_line;
            }
            // is not or is end of row
            if(this->U_map.at<uchar>(row,col) < u_min || col == this->U_map.cols - 1)
            {
                // is end of the row
                col = (col == this->U_map.cols - 1)? col + 1:col;
                // is good line candidate (length and sum)
                if(length_line > this->min_length_line && sum_line > this->threshold_line * max_line)
                {
                    seg_id++;
                    UVboxes.push_back(UVbox(seg_id, row, col - length_line, col - 1));
                    // overwrite the mask with segementation id
                    for(int c = col - length_line; c < col - 1; c++)
                    {
                        mask[row][c] = seg_id;
                    }
                    // when current row is not first row, we need to merge neighbour segementation
                    if(row != 0)
                    {
                        // merge all parents
                        for(int c = col - length_line; c < col - 1; c++)
                        {
                            if(mask[row - 1][c] != 0)
                            {
                                if(UVboxes[mask[row - 1][c] - 1].toppest_parent_id < UVboxes.back().toppest_parent_id)
                                {
                                    UVboxes.back().toppest_parent_id = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                }
                                else
                                {
                                    int temp = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                    for(int b = 0; b < UVboxes.size(); b++)
                                    {
                                        UVboxes[b].toppest_parent_id = (UVboxes[b].toppest_parent_id == temp)?UVboxes.back().toppest_parent_id:UVboxes[b].toppest_parent_id;
                                    }
                                }
                            }
                        }
                    }
                }
                sum_line = 0;
                max_line = 0;
                length_line = 0;
            }
        }
    }
    
    // group lines into boxes
    this->bounding_box_U.clear();
    // merge boxes with same toppest parent
    for(int b = 0; b < UVboxes.size(); b++)
    {
        if(UVboxes[b].id == UVboxes[b].toppest_parent_id)
        {
            for(int s = b + 1; s < UVboxes.size(); s++)
            {
                if(UVboxes[s].toppest_parent_id == UVboxes[b].id)
                {
                    UVboxes[b] = merge_two_UVbox(UVboxes[b], UVboxes[s]);

                }
            }
            // check box's size
            if(UVboxes[b].bb.area() >= 25)
            {
                this->bounding_box_U.push_back(UVboxes[b].bb);
            }
        }
    }
}

void UVdetector::detect()
{
    // Fit and remove ground plane
    this->fitGroundPlane();
    this->removeGroundPoints();

    // extract U map from depth
    this->extract_U_map();

    // extract bounding box from U map
    this->extract_bb();

    // extract bounding box
    this->extract_bird_view();

    // extract object's height
    // this->extract_height();
}

void UVdetector::display_depth()
{
    // 1. Create a displayable image (8-bit BGR)
    Mat depthDisplay;
    Mat depth8U;

    // Check input depth type and convert to 8-bit for display
    if (this->depth.type() == CV_16UC1) {
         // Scale valid depth range [min_dist, max_dist] to [0, 255]
         double scaleFactor = 255.0 / (this->max_dist - this->min_dist);
         // Apply scaling, handling invalid ranges
         if (this->max_dist > this->min_dist) {
            this->depth.convertTo(depth8U, CV_8UC1, scaleFactor, -this->min_dist * scaleFactor);
         } else {
             // Avoid division by zero if min/max dist are bad
             this->depth.convertTo(depth8U, CV_8UC1); // Simple conversion, might not look good
         }
         // Set pixels originally outside the valid range or zero to black (0)
         depth8U.setTo(0, (this->depth < this->min_dist) | (this->depth > this->max_dist) | (this->depth == 0));
    } else if(this->depth.type() == CV_8UC1) {
        // If already 8-bit, clone it
        depth8U = this->depth.clone();
    } else {
        // Handle unexpected types
        cerr << "Warning: Unexpected depth image type (" << this->depth.type() << ") in display_depth(). Displaying black image." << endl;
        depth8U = Mat::zeros(this->depth.rows, this->depth.cols, CV_8UC1);
    }

    // Convert the 8-bit grayscale image to BGR for drawing colored boxes
    // You might prefer a colormap for better depth perception
    cvtColor(depth8U, depthDisplay, COLOR_GRAY2BGR);
    // Example: applyColorMap(depth8U, depthDisplay, COLORMAP_JET);
    // If using colormap, you might want to keep invalid pixels black:
    // if (depthDisplay.channels() == 3) { // Ensure it's color after colormap
    //    depthDisplay.setTo(Scalar(0,0,0), depth8U == 0);
    // }

    // 2. Loop through tracked objects (using Kalman filter estimates)
    for (int i = 0; i < this->tracker.now_filter.size(); ++i)
    {
        // Add a check here if your Kalman filter class has it:
        // if (!this->tracker.now_filter[i].is_initialized()) continue;

        // 3. Get estimated state in bird's eye view coords from Kalman filter
        // IMPORTANT: Confirm these state vector indices are correct for YOUR filter!
        // Assuming State: [Xc, Zc, Vxc, Vzc, Width_X, Depth_Z]
        // Xc = Center X (metric, perpendicular to camera axis)
        // Zc = Center Z (metric, depth along camera axis)
        // Width_X = Metric width (in X dimension)
        float Xc = this->tracker.now_filter[i].output(0);
        float Zc = this->tracker.now_filter[i].output(1);
        float W_x = this->tracker.now_filter[i].output(4);
        // float D_z = this->tracker.now_filter[i].output(5); // Metric depth extent - not used directly for 2D box height here

        // Basic check for valid depth (prevent division by zero/small numbers)
        // Adjust the threshold (e.g., 1.0mm or 0.001m depending on units)
        if (Zc < 1.0f) {
            continue;
        }

        // 4. Project the 3D Center Point (Xc, Y=0, Zc) to 2D Image Point (uc, vc)
        // Assuming Y=0 (object base on ground relative to camera) for projection simplicity
        float Y_base = 0.0f;
        float uc = this->fx * (Xc / Zc) + this->px;
        // Projecting Y=0 puts the vertical center at the principal point height (py)
        float vc = this->fy * (Y_base / Zc) + this->py;

        // 5. Estimate Pixel Width and Height for the 2D Box
        // Pixel width approx = focal_length_x * (Metric Width / Metric Depth)
        float w_pix = this->fx * (W_x / Zc);

        // Estimate Pixel Height based on pixel width and an assumed aspect ratio
        // *** TUNE THIS ASPECT RATIO based on typical objects you detect ***
        // e.g., 1.0 for square-ish, 1.5-2.0 for people (taller), 0.5-0.8 for cars (wider)
        float aspect_ratio = 1.8f; // Example: Assume objects are generally taller than wide
        float h_pix = w_pix * aspect_ratio;

        // --- Alternative: Fixed size boxes ---
        // float fixed_size = 40.0f; // Size in pixels
        // w_pix = fixed_size;
        // h_pix = fixed_size;
        // --- End Alternative ---

        // 6. Calculate Top-Left (tl) and Bottom-Right (br) pixel coordinates for the 2D box
        // Centering the box around the projected point (uc, vc)
        float tl_u = uc - w_pix / 2.0f;
        float tl_v = vc - h_pix / 2.0f;
        float br_u = uc + w_pix / 2.0f;
        float br_v = vc + h_pix / 2.0f;

        // 7. Clip coordinates to image boundaries
        tl_u = std::max(0.0f, tl_u);
        tl_v = std::max(0.0f, tl_v);
        br_u = std::min((float)depthDisplay.cols - 1.0f, br_u);
        br_v = std::min((float)depthDisplay.rows - 1.0f, br_v);

        // 8. Ensure box has valid dimensions (width > 0, height > 0) before drawing
         if (br_u > tl_u && br_v > tl_v) {
             // Draw the 2D rectangle on the color display image
             rectangle(depthDisplay,
                       Point(cvRound(tl_u), cvRound(tl_v)), // Top-left corner
                       Point(cvRound(br_u), cvRound(br_v)), // Bottom-right corner
                       Scalar(0, 255, 0), // Color (Green)
                       2);                // Thickness
         }
    }

    // 9. Show the final image with depth visualization and 2D boxes
    imshow("Depth", depthDisplay);
    waitKey(1);
}

void UVdetector::display_U_map()
{
    // visualize with bounding box
    if(this->show_bounding_box_U)
    {
        this->U_map = this->U_map * 10;
        cvtColor(this->U_map, this->U_map, cv::COLOR_GRAY2RGB);
        for(int b = 0; b < this->bounding_box_U.size(); b++)
        {
            Rect final_bb = Rect(this->bounding_box_U[b].tl(),Size(this->bounding_box_U[b].width, 2 * this->bounding_box_U[b].height));
            rectangle(this->U_map, final_bb, Scalar(0, 0, 255), 1, 8, 0);
            circle(this->U_map, Point2f(this->bounding_box_U[b].tl().x + 0.5 * this->bounding_box_U[b].width, this->bounding_box_U[b].br().y ), 2, Scalar(0, 0, 255), 5, 8, 0);
        }
    } 
    
    imshow("U map", this->U_map);
    waitKey(1);
}

void UVdetector::extract_bird_view()
{
    // extract bounding boxes in bird's view
    uint8_t histSize = this->depth.rows / this->row_downsample;
    uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
    this->bounding_box_B.clear();
    this->bounding_box_B.resize(this->bounding_box_U.size());

    for(int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // U_map bounding box -> Bird's view bounding box conversion
        float bb_depth = this->bounding_box_U[b].br().y * bin_width / 10;
        float bb_width = bb_depth * this->bounding_box_U[b].width / this->fx;
        float bb_height = this->bounding_box_U[b].height * bin_width / 10;
        float bb_x = bb_depth * (this->bounding_box_U[b].tl().x / this->col_scale - this->px) / this->fx;
        float bb_y = bb_depth - 0.5 * bb_height;
        this->bounding_box_B[b] = Rect(bb_x, bb_y, bb_width, bb_height);
    }

    // initialize the bird's view
    this->bird_view = Mat::zeros(BIRD_VIEW_HEIGHT, BIRD_VIEW_WIDTH, CV_8UC1);
    cvtColor(this->bird_view, this->bird_view, cv::COLOR_GRAY2RGB);
}

void UVdetector::display_bird_view()
{
    // center point
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    Point2f left_end_to_center = Point2f( this->bird_view.rows * (0 - this->px) / this->fx, -this->bird_view.rows);
    Point2f right_end_to_center = Point2f( this->bird_view.rows * (this->depth.cols - this->px) / this->fx, -this->bird_view.rows);

    // draw the two side lines
    line(this->bird_view, center, center + left_end_to_center, Scalar(0, 255, 0), 3, 8, 0);
    line(this->bird_view, center, center + right_end_to_center, Scalar(0, 255, 0), 3, 8, 0);

    for(int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // change coordinates
        Rect final_bb = this->bounding_box_B[b];
        final_bb.y = center.y - final_bb.y - final_bb.height;
        final_bb.x = final_bb.x + center.x; 
        // draw center 
        Point2f bb_center = Point2f(final_bb.x + 0.5 * final_bb.width, final_bb.y + 0.5 * final_bb.height);
        rectangle(this->bird_view, final_bb, Scalar(0, 0, 255), 3, 8, 0);
        circle(this->bird_view, bb_center, 3, Scalar(0, 0, 255), 5, 8, 0);
    }

    // show
    resize(this->bird_view, this->bird_view, Size(), BIRD_VIEW_SCALE, BIRD_VIEW_SCALE);
    imshow("Bird's View", this->bird_view);
    waitKey(1);
}

void UVdetector::add_tracking_result()
{
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    for(int b = 0; b < this->tracker.now_bb.size(); b++)
    {
        // change coordinates
        Point2f estimated_center = Point2f(this->tracker.now_filter[b].output(0), this->tracker.now_filter[b].output(1));
        estimated_center.y = center.y - estimated_center.y;
        estimated_center.x = estimated_center.x + center.x; 
        // draw center 
        circle(this->bird_view, estimated_center, 5, Scalar(0, 255, 0), 5, 8, 0);
        // draw bounding box
        Point2f bb_size = Point2f(this->tracker.now_filter[b].output(4), this->tracker.now_filter[b].output(5));
        rectangle(this->bird_view, Rect(estimated_center - 0.5 * bb_size, estimated_center + 0.5 * bb_size), Scalar(0, 255, 0), 3, 8, 0);
        // draw velocity
        Point2f velocity = Point2f(this->tracker.now_filter[b].output(2), this->tracker.now_filter[b].output(3));
        velocity.y = -velocity.y;
        line(this->bird_view, estimated_center, estimated_center + velocity, Scalar(255, 255, 255), 3, 8, 0);
        for(int h = 1; h < this->tracker.now_history[b].size(); h++)
        {
            // trajectory
            Point2f start = this->tracker.now_history[b][h - 1];
            start.y = center.y - start.y;
            start.x = start.x + center.x;
            Point2f end = this->tracker.now_history[b][h];
            end.y = center.y - end.y;
            end.x = end.x + center.x;
            line(this->bird_view, start, end, Scalar(0, 0, 255), 3, 8, 0);
        }
    }
}

void UVdetector::track()
{
    this->tracker.read_bb(this->bounding_box_B);
    this->tracker.check_status();
    this->add_tracking_result();
}

