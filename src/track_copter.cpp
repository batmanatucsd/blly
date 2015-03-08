#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>

#include <cmath>
#include <vector>

#include <boost/thread/mutex.hpp>

#define CROP_WIDTH 320
#define CROP_HEIGHT 240

#define THRESH_MAG 3

#define RANGE_MAX 9.757 //TODO find real max based on range of sensor (mm or meters)
#define EDGE_LOW_MAX 100
#define EDGE_HIGH_MAX 100

#define DEPTH_SCALE_FACTOR_MAX 100
#define NUM_FUNCTIONS 2
#define BLUR_SIZE_MAX 100
//if 1, contours must be square. smaller value allows for skinnier contours
#define CONTOUR_ECCENTRICTY_THRESH 0.3 //TODO look into the value for best approx
#define IR_CAMERA_FOCAL_PX 580 //(focal point in pixels) //taken from wiki.ros.org/kinect_calibration/technical
#define IR_CAMERA_FOCAL_PY 580 //TODO find actual focal length and calculate PX and PY based on pixel width and height (do not assume square pixel)

#define SCALE_AREA_MIN 0.0001
#define SCALE_AREA_MAX 0.04

using namespace std;
using namespace cv;


// Forward Declaration
static void drawMotionIntensity(const cv::Mat&, cv::Mat&);
static void drawOptFlowMap(const cv::Mat&, cv::Mat&,
        int, double, const cv::Scalar&);
static void drawRectsFromContours(cv::Mat&,
        std::vector<std::vector<cv::Point> >&);
static void filterContoursByShape(std::vector<std::vector<cv::Point> > &, std::vector<std::vector<cv::Point> > &, float);
static void filterContoursByScale(std::vector<std::vector<cv::Point> > &, std::vector<std::vector<cv::Point> > &, const cv::Mat&, const cv::Mat&);
double findContourDepth(const cv::Mat &);
double calcApproxSurfaceArea(const cv::Mat &, double);

//get ride of erroneous black values
//these happen by noise or when there is no surface reflecting back kinect IR
static void threshBlack(const cv::Mat&, cv::Mat&);

static void copyOutBlack(const cv::Mat&, cv::Mat&, float);
static void interpOutBlack(const cv::Mat&, cv::Mat&, float);
static void interp1D(cv::Mat&, int, int, int, int, int, int);

static boost::mutex mutex;

class CopterTracker
{
    //cv_bridge::CvImagePtr rgb_orig;
    bool rgb_locked_writing;
    bool rgb_locked_reading;
    bool shared_contours_finalists_writing;
    bool shared_contours_finalists_reading;

    image_transport::ImageTransport iTrans;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub_depth;
    image_transport::Subscriber image_sub_rgb;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > contours_cand;
    std::vector<std::vector<cv::Point> > contours_finalists;
    std::vector<std::vector<cv::Point> > shared_contours_finalists;
    std::vector<std::vector<cv::Point> > contours_finalists_rgb;

    void callback_depth(const sensor_msgs::ImageConstPtr& msg);
    void callback_rgb(const sensor_msgs::ImageConstPtr& msg);

    public:
    int edge_thresh_low;
    int edge_thresh_high;
    int depth_scale_factor;
    int function_toggle;
    int blur_size;

    cv::Mat kernel;

    CopterTracker(ros::NodeHandle n): iTrans(n)
    {
        //TODO possibly subscribe to rectified depth to improve reprojection accuracy
        image_sub_depth = iTrans.subscribe("/camera/depth/image", 1, &CopterTracker::callback_depth, this);
        image_sub_rgb = iTrans.subscribe("/camera/rgb/image_color", 1, &CopterTracker::callback_rgb, this);

        image_pub = iTrans.advertise("/camera/depth_cleaned/image", 1);

        edge_thresh_low = 32;
        edge_thresh_high = 46;
        depth_scale_factor = 100/8;
        function_toggle = 0;
        blur_size = 7;
        kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(-1,-1));
        rgb_locked_writing = false;
        rgb_locked_reading = false;
        bool shared_contours_finalists_writing = false;
        bool shared_contours_finalists_reading = false;

        //algorithm_mode = FOFA; //intializes the algorithm to FOFA
    }

};

void CopterTracker::callback_depth(const sensor_msgs::ImageConstPtr& msg)
{   
    /*
    cv::Mat gray_img;
    //mutex.lock();
    while(1){
        if (rgb_locked_writing) continue;
        rgb_locked_reading = true;
        cvtColor(rgb_orig->image, gray_img, CV_RGB2GRAY);
        rgb_locked_reading = false;
        //mutex.unlock();
        break;
    }
    */

    cv_bridge::CvImagePtr depth_orig;

//    std::string label;

    try {
        depth_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::waitKey(0);

    double min;
    double max;
    minMaxLoc(depth_orig->image, &min, &max);
    //ROS_INFO("\nmin: %lf\nmax: %lf", min, max);

    //TODO take advantage of decimal pixel data before converting to uint8
    //take depth frames centered at some depth d, with width w. then convert those frames to uint8
    //sliding depth frme (focus on a depth range) and use histEQ on each slice

    //TODO investigate a tracking algorithm to speed up detection and searching

    //convert to grayscale and make a copy
    cv::Mat depth_img;
    depth_orig->image.convertTo(depth_img, CV_8UC1, 255/RANGE_MAX, 0);//scale by the max value and shift by 0



    cv::Mat depth_img_cleaned = depth_img.clone(); //copy depth_img
    //clean up the depth image by making erroneous black pixels white
    //threshBlack(depth_img, depth_img_cleaned);
/*    for(int i = 0; i < depth_img.rows; i++) {
        for(int j = depth_img.cols-10; j < depth_img.cols; j++) {
            depth_img.at<uchar>(i, j) = 255;

        }
    }*/
    if (function_toggle == 0){
        copyOutBlack(depth_img, depth_img_cleaned, float(depth_scale_factor)/DEPTH_SCALE_FACTOR_MAX*8);
    }
    else if (function_toggle == 1){
        interpOutBlack(depth_img, depth_img_cleaned, float(depth_scale_factor)/DEPTH_SCALE_FACTOR_MAX*8);
    }

    cv::GaussianBlur(depth_img_cleaned, depth_img_cleaned, cv::Size(2+blur_size+!(blur_size%2), 2+blur_size+!(blur_size%2)), 9, cv::BORDER_DEFAULT);

    Mat depth_img_edges(depth_img_cleaned.rows, depth_img_cleaned.cols, CV_8UC1);
    Canny(depth_img_cleaned, depth_img_edges, edge_thresh_low, edge_thresh_high);


    //make edges thicker to help find better contours
    cv::dilate(depth_img_edges, depth_img_edges, kernel, cv::Point(-1,-1), 4);
    cv::findContours(depth_img_edges.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    contours_cand.clear();
    //filters based on shape
    filterContoursByShape(contours_cand, contours, CONTOUR_ECCENTRICTY_THRESH);

    Mat contour_img = Mat::zeros(depth_img_edges.rows, depth_img_edges.cols, CV_8UC1);
    drawRectsFromContours(contour_img, contours_cand);

    cv::waitKey(1);
    imshow("depth", depth_img);
    imshow("Depth: Cleaned-Up", depth_img_cleaned);
    imshow("Depth: Edges", depth_img_edges);

    imshow("Candidate Contours: filtered by shape", contour_img);

    //Mat test_img = Mat::zeros(depth_img_edges.rows, depth_img_edges.cols, CV_8UC1);
    //drawContours(test_img, contours_cand, -1, 255, CV_FILLED);
    //imshow("test", test_img);

    contours_finalists.clear();
    //filter based on scale (should be rough size of copter)
    filterContoursByScale(contours_finalists, contours_cand, depth_orig->image, kernel); //TODO pass in depth_orig

    Mat contour_img_finalists = Mat::zeros(depth_img_edges.rows, depth_img_edges.cols, CV_8UC1);
    drawRectsFromContours(contour_img_finalists, contours_finalists);

    imshow("contour finalists: filtered by shape and scale", contour_img_finalists);

    if (!shared_contours_finalists_reading) {
        shared_contours_finalists_writing = true;
        shared_contours_finalists = contours_finalists;
        shared_contours_finalists_writing = false;
    }


    //blur the image before applying algorithm
    //cv::GaussianBlur(frame_gray, frame_gray, cv::Size(7, 7), 7, cv::BORDER_DEFAULT);
/*
    switch (algorithm_mode) {
        case FOFA:
            if (!prev.empty()){
                cv::calcOpticalFlowFarneback(prev, frame_gray, uflow, 0.5, 2, 3, 2, 5, 1.1, 0);
                cv::cvtColor(prev, cflow, cv::COLOR_GRAY2BGR);
                uflow.copyTo(flow);
                //draws arrows over the prev grayscale frames (cflow)
                drawOptFlowMap(flow, cflow, 16, 1.5, cv::Scalar(0, 255, 0));
                //imshow(FLOW_WINDOW_ARROWS, cflow);
                cv::Mat intensityMap = cv::Mat::zeros(flow.rows, flow.cols, CV_8U);
                //populates the intensityMap with the binary image for flow
                drawMotionIntensity(flow, intensityMap);
                //remove white specks (noise)
                cv::erode(intensityMap, intensityMap, kernel, cv::Point(-1,-1), 7);
                //dilate what's left to help find blob of body
                cv::dilate(intensityMap, intensityMap, kernel, cv::Point(-1,-1), 30);
                cv::findContours(intensityMap, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                //draw rectanle on the original image
                drawRectFromContours(frame_original_ptr->image, contours);
                //drawContours(intensityMap, contours, -1, cv::Scalar(0,0,255), 3);
                //rectangle(intensityMap, Point(5, 5), Point(100,100), Scalar(255,255,255), 5);
                //imshow(FLOW_WINDOW_BINARY, intensityMap);
            }
            label.clear();
            label.append(LABEL_FOFA);
            break;
        case MOG2:
            bsmog(frame_gray, fg_mask, -1);
            bsmog.set("nmixtures", 3);
            cv::erode(fg_mask, fg_mask, kernel, cv::Point(-1,-1), 10);
            cv::dilate(fg_mask, fg_mask, kernel, cv::Point(-1,-1), 32);
            cv::findContours(fg_mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
            std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
            std::vector<cv::Rect> boundRect( contours.size() );
            for( int i = 0; i < contours.size(); i++ ) {
                cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
                boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
                cv::rectangle(frame_original_ptr->image, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 0, 0), 2, 8, 0 );
            }
            //cv::imshow(MOG2_WINDOW, fg_mask);
            label.clear();
            label.append(LABEL_MOG2);
            break;
    }
*/


    // Add algorithm label to the image
/*
    rectangle(frame_original_ptr->image, cv::Point(10, 10), cv::Point(150,50),
            cv::Scalar(255,255,255), -1);
    putText(frame_original_ptr->image, label, cv::Point(25, 45),
            cv::FONT_HERSHEY_SIMPLEX, 1.5 , cv::Scalar(0,0,0));

    prev = frame_gray.clone();
    image_pub.publish(frame_original_ptr->toImageMsg());

    // Reset Contours
    contours.clear();
*/
}

void CopterTracker::callback_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("got rgb image");
    cv_bridge::CvImagePtr rgb_orig;
   /*
    try {
        //TODO make sure mutex is working
        //mutex.lock();
        while(1){
            if (rgb_locked_reading) continue;
            rgb_locked_writing = true;
            rgb_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
            //mutex.unlock();
            rgb_locked_writing = false;
            break;
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }*/

    try {
        rgb_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat gray_img;
    cvtColor(rgb_orig->image, gray_img, CV_RGB2GRAY);

    //TODO mutex lock to make this if statement obsolete
    if (!shared_contours_finalists_writing){

        shared_contours_finalists_reading = true;
        contours_finalists_rgb = shared_contours_finalists;
        shared_contours_finalists_reading = false;

        //check to make sure it is not accessed before populated
        //if (contours_finalists_rgb.empty()) return;

        //Mat contour_img_finalists = Mat::zeros(gray_img.rows, gray_img.cols, CV_8UC1);
        //drawRectsFromContours(contour_img_finalists, contours_finalists_rgb);
        drawRectsFromContours(gray_img, contours_finalists_rgb);

    }

    imshow("gray image: contours finalists", gray_img);


}

static void threshBlack(const cv::Mat& depth, cv::Mat& depth_cleaned){

    double min;
    double max;

    minMaxLoc(depth, &min, &max);

    //ROS_INFO("Min,Max: %lf, %lf", min, max);

    for(int i = 0; i < depth.rows; i++) {
        for(int j = 0; j < depth.cols; j++) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int pixel = depth.at<uchar>(i, j);
            if (pixel == 0 || pixel < 5){
                //opImg.at<float>(1, 1) = 255;
                depth_cleaned.at<uchar>(i,j) = 255;
                //depth.data[depth.step[0]*i + depth.step[1]*j + 0] = 255;
                //depth_cleaned.data[depth_cleaned.step[0]*i + depth_cleaned.step[1]*j + 0] = 255;
            }
            //ROS_INFO("rows: %d", opImg.rows);
            //ROS_INFO("cols: %d", opImg.cols);
            //ROS_INFO("Pixel at (%d,%d): %d", j,i,pixel);
            //ROS_INFO("depth Dims: %d", depth.dims);
            //ROS_INFO("depth_cleaned Dims: %d", depth_cleaned.dims);

        }
    }

}

static void copyOutBlack(const cv::Mat& depth, cv::Mat& depth_cleaned, float depth_scale_factor){

    //TODO interpolate values

    Mat left_to_right = depth_cleaned.clone();
    Mat right_to_left = depth_cleaned.clone();
    Mat top_to_bottom = depth_cleaned.clone();
    Mat bottom_to_top = depth_cleaned.clone();
    int pre = 0;

    //left to right
    //find black, erreneous pixels and copy the previous pixel over it
    for(int i = 0; i < depth.rows; i++) {
        //set pre for the row
        for(int j = 0; j < depth.cols; j++) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int j = 0; j < depth.cols; j++) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                left_to_right.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                continue; //do not update pre, other
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);

        }
    }

    //right to left
    //find black, erreneous pixels and copy the previous pixel over it
    for(int i = depth.rows-1; i >= 0; i--) {
        //set pre for the row
        for(int j = depth.cols-1; j >= 0; j--) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int j = depth.cols-1; j >= 0; j--) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                right_to_left.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                continue; //do not update pre, other
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);

        }
    }


    //top to bottom
    //find black, erreneous pixels and copy the previous pixel over it
    for(int j = 0; j < depth.cols; j++) {
        //set pre for the col
        for(int i = 0; i < depth.rows; i++) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int i = 0; i < depth.rows; i++) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                top_to_bottom.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                continue; //do not update pre, other
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);

        }
    }

    //bottom_to_top
    //find black, erreneous pixels and copy the previous pixel over it
    for(int j = depth.cols-1; j >= 0; j--) {
        //set pre for the col
        for(int i = depth.rows-1; i >= 0; i--) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int i = depth.rows-1; i >= 0; i--) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                bottom_to_top.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                continue; //do not update pre, other
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);

        }
    }

    //depth_cleaned = (left_to_right+right_to_left)/2;
    //depth_cleaned = right_to_left;
    //depth_cleaned = left_to_right;
    //depth_cleaned = bottom_to_top;
    //depth_cleaned = top_to_bottom;
    depth_cleaned = (left_to_right+right_to_left+top_to_bottom+bottom_to_top)/depth_scale_factor;
}

//interpolate from p1 to p2
static void interp1D(cv::Mat& opImg, int p1, int p2, int row1, int col1, int row2, int col2){

    int pre = p1;
    int end = p2;
    int togo; //number of places to interp
    int curr;
    int row, col;

    //determine direction
    //horizontal
    if (row1 == row2){
        row = row1;
        col = col1;
        //left to right
        if (col2 > col1){
            //col = col1;
            togo = col2 - col1 - 1;
            while (togo > 0){
                col++;
                curr = pre + (end - pre)/(togo + 1);
                opImg.at<uchar>(row,col) = curr;
                togo--;
                pre = curr;
            }
        }
        //right to left
        else {
            togo = col1 - col2 - 1;
            while (togo > 0){
                col--;
                curr = pre + (end - pre)/(togo + 1);
                opImg.at<uchar>(row,col) = curr;
                togo--;
                pre = curr;
            }
        }

    }
    //vertical
    else if (col1 == col2){
        row = row1;
        col = col1;
        //top to bottom
        if (row2 > row1){

            togo = row2 - row1 - 1;
            while (togo > 0){
                row++;
                curr = pre + (end - pre)/(togo + 1);
                opImg.at<uchar>(row,col) = curr;
                togo--;
                pre = curr;
            }

        }
        //bottom to top
        else {
            togo = row1 - row2 - 1;
            while (togo > 0){
                row--;
                curr = pre + (end - pre)/(togo + 1);
                opImg.at<uchar>(row,col) = curr;
                togo--;
                pre = curr;
            }

        }

    }


}

static void interpOutBlack(const cv::Mat& depth, cv::Mat& depth_cleaned, float depth_scale_factor){

    //TODO interpolate values

    Mat left_to_right = depth_cleaned.clone();
    Mat right_to_left = depth_cleaned.clone();
    Mat top_to_bottom = depth_cleaned.clone();
    Mat bottom_to_top = depth_cleaned.clone();
    int pre = 0;
    int zeroCount = 0;
    int i1, j1, i2, j2;

    //interpolate from p1 to p2
    int p1, p2;

    //left to right
    //find black, erreneous pixels and copy the previous pixel over it
    for(int i = 0; i < depth.rows; i++) {
        //set pre for the row
        for(int j = 0; j < depth.cols; j++) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int j = 0; j < depth.cols; j++) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                zeroCount++;
                //if entering a black region
                //assign p1 the pixel value before the start of the black region
                if (zeroCount == 1){
                    p1 = pre;
                    i1 = i;
                    j1 = j-1;//-1 since to get the previous index, not curr
                }
                //else {
                //    p2 = curr;
                //}

                //left_to_right.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                //continue; //do not update pre, other
            }

            //if leaving a black region
            //interpolate from p1 to p2
            else if (zeroCount > 0){
                zeroCount = 0;
                p2 = curr;
                i2 = i;
                j2 = j;
                //do not interpolate if on an edge, rather copy over black
                if (i1 < 0 || j1 < 0) {
                    //ROS_INFO("(%d,%d): WARNING, negative indexing", i1, j1);

                }
                else {
                    interp1D(left_to_right, p1, p2, i1, j1, i2, j2);
                }
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);

        }
    }

    //right to left
    //find black, erreneous pixels and copy the previous pixel over it
    for(int i = depth.rows-1; i >= 0; i--) {
        //set pre for the row
        for(int j = depth.cols-1; j >= 0; j--) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int j = depth.cols-1; j >= 0; j--) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                zeroCount++;
                //if entering a black region
                //assign p1 the pixel value before the start of the black region
                if (zeroCount == 1){
                    p1 = pre;
                    i1 = i;
                    j1 = j+1;//+1 since to get the previous index, not curr
                }
                //else {
                //    p2 = curr;
                //}

                //left_to_right.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                //continue; //do not update pre, other
            }

            //if leaving a black region
            //interpolate from p1 to p2
            else if (zeroCount > 0){
                zeroCount = 0;
                p2 = curr;
                i2 = i;
                j2 = j;
                interp1D(right_to_left, p1, p2, i1, j1, i2, j2);
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);
        }
    }


    //top to bottom
    //find black, erreneous pixels and copy the previous pixel over it
    for(int j = 0; j < depth.cols; j++) {
        //set pre for the col
        for(int i = 0; i < depth.rows; i++) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int i = 0; i < depth.rows; i++) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                zeroCount++;
                //if entering a black region
                //assign p1 the pixel value before the start of the black region
                if (zeroCount == 1){
                    p1 = pre;
                    i1 = i-1;//-1 since to get the previous index, not curr
                    j1 = j;
                }
                //else {
                //    p2 = curr;
                //}

                //left_to_right.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                //continue; //do not update pre, other
            }

            //if leaving a black region
            //interpolate from p1 to p2
            else if (zeroCount > 0){
                zeroCount = 0;
                p2 = curr;
                i2 = i;
                j2 = j;
                interp1D(top_to_bottom, p1, p2, i1, j1, i2, j2);
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);
        }
    }

    //bottom to top
    //find black, erreneous pixels and copy the previous pixel over it
    for(int j = depth.cols-1; j >= 0; j--) {
        //set pre for the col
        for(int i = depth.rows-1; i >= 0; i--) {
            if (depth.at<uchar>(i, j) != 0){
                pre = depth.at<uchar>(i, j);
                break;
            }
        }
        for(int i = depth.rows-1; i >= 0; i--) {
            //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
            const int curr = depth.at<uchar>(i, j);
            if (curr == 0){
                zeroCount++;
                //if entering a black region
                //assign p1 the pixel value before the start of the black region
                if (zeroCount == 1){
                    p1 = pre;
                    i1 = i+1;//+1 since to get the previous index, not curr
                    j1 = j;
                }
                //else {
                //    p2 = curr;
                //}

                //left_to_right.at<uchar>(i,j) = pre;
                //ROS_INFO("pres: %d", pre);
                //continue; //do not update pre, other
            }

            //if leaving a black region
            //interpolate from p1 to p2
            else if (zeroCount > 0){
                zeroCount = 0;
                p2 = curr;
                i2 = i;
                j2 = j;
                interp1D(bottom_to_top, p1, p2, i1, j1, i2, j2);
            }

            pre = curr;
            //ROS_INFO("rows: %d", opImg.rows);
        }
    }


    //depth_cleaned = (left_to_right+right_to_left)/2;
    //depth_cleaned = right_to_left;
    //depth_cleaned = left_to_right;
    //depth_cleaned = bottom_to_top;
    //depth_cleaned = top_to_bottom;
    depth_cleaned = (left_to_right+right_to_left+top_to_bottom+bottom_to_top)/depth_scale_factor;
}

static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
        double, const cv::Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step) {
        for(int x = 0; x < cflowmap.cols; x += step) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            cv::line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                    color);
            cv::circle(cflowmap, cv::Point(x,y), 2, color, -1);
        }
    }
}

static void drawMotionIntensity(const cv::Mat& flow, cv::Mat& A)
{
    for(int i = 0; i < flow.rows; i++) {
        for(int j = 0; j < flow.cols; j++) {
            const cv::Point2f& v = flow.at<cv::Point2f>(i, j);
            const int mag = sqrt(v.x*v.x + v.y*v.y);
            //line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
            //     color);
            //circle(cflowmap, Point(x,y), 2, color, -1);
            //int x = flow.data[flow.step[0]*i + flow.step[1]*j + 0];
            //int y = flow.data[flow.step[0]*i + flow.step[1]*j + 1];
            //std::cout << "x: " << x << std::endl;
            //std::cout << "y: " << y << std::endl;
            //A.data[A.step[0]*i + A.step[1]*j + 0] = sqrt(x*x + y*y);
            if (mag > THRESH_MAG){
                A.data[A.step[0]*i + A.step[1]*j + 0] = 255;
            }
        }
    }
}

static void drawRectsFromContours(cv::Mat& frame, std::vector<std::vector<cv::Point> > &contours)
{
    if (contours.empty()) { return; }

    //draw all contours as boxes
    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin() ; it != contours.end(); ++it){    
        cv::rectangle(frame, boundingRect(*it), 255, 5);//cv::Point(minX, minY), cv::Point(maxX,maxY)

    }

}

static void filterContoursByScale(std::vector<std::vector<cv::Point> > &contours_finalists, std::vector<std::vector<cv::Point> > &contours_cand, const cv::Mat& depth_img_orig, const cv::Mat& kernel)
{
    if (contours_cand.empty()) { return; }

    float depth_average;
    double scale_area;
    cv::Scalar temp;
    cv::Mat contour_mask = Mat::zeros(depth_img_orig.rows, depth_img_orig.cols, CV_8UC1);
    std::vector<std::vector<cv::Point> > single_contour;

    //search through all candidate contours
    for (std::vector<std::vector<cv::Point> >::iterator it = contours_cand.begin() ; it != contours_cand.end(); ++it){

        if (it->empty()) continue;
        contour_mask = Mat::zeros(depth_img_orig.rows, depth_img_orig.cols, CV_8UC1);

        //get contour mask
        single_contour.push_back(*it);
        drawContours(contour_mask, single_contour, -1, 1, CV_FILLED);

        //erode to get exactly where the original contour was found (undo the dilation and try to leave the largest part)
        cv::erode(contour_mask, contour_mask, kernel, cv::Point(-1,-1), 4+4);

        imshow("contour mask", contour_mask*255);
        cv::Mat contour_depth_F(depth_img_orig.rows, depth_img_orig.cols, CV_32FC1);
        cv::Mat contour_mask_F(depth_img_orig.rows, depth_img_orig.cols, CV_32FC1);

        contour_mask.convertTo(contour_mask_F, CV_32FC1);
        contour_depth_F = depth_img_orig.mul(contour_mask_F);
        //cv::Mat buff_U(depth_img_orig.rows, depth_img_orig.cols, CV_8UC1);
        //depth_img_masked_F.convertTo(buff_U, CV_8UC1);

        depth_average = findContourDepth(contour_depth_F);

        //ROS_INFO("sum: %lf", sum);
        //ROS_INFO("count: %d", count);


        //TODO filter based on scale, not just depth
        if (depth_average > 0){

            scale_area = calcApproxSurfaceArea(contour_depth_F, depth_average);

            //ROS_INFO("depth: %.6f", depth_average);
            //ROS_INFO("area: %.6f\n", scale_area);

            if (scale_area >= SCALE_AREA_MIN && scale_area <= SCALE_AREA_MAX){
                contours_finalists.push_back(*it);
                ROS_INFO("valid area: %.6f\n", scale_area);
            }

        }
        single_contour.pop_back();
    }


}

//find good contours given all contours as input
static void filterContoursByShape(std::vector<std::vector<cv::Point> > &good_contours, std::vector<std::vector<cv::Point> > &contours, float ratioThresh)
{
    if (contours.empty()) { return; }

    cv::Rect rect;
    float ratio;
    std::vector<cv::Point> contour;

    //search through all contours
    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin() ; it != contours.end(); ++it){

        if (it->empty()) continue;
        contour = *it;
        rect = boundingRect(contour);

        if (rect.width == 0 || rect.height == 0) continue;

        (rect.width > rect.height) ? (ratio = float(rect.height)/rect.width) : (ratio = float(rect.width)/rect.height);

        //only add contours which are not too skinny or tall to good_contours
        if (ratio > ratioThresh){
            good_contours.push_back(contour);

        }

    }
}

double findContourDepth(const cv::Mat &contour_depth_F)
{
    double depth;
    double min;
    double max;
    minMaxLoc(contour_depth_F, &min, &max);
    cv::Mat contour_depth_U(contour_depth_F.rows, contour_depth_F.cols, CV_8UC1);
    //ROS_INFO("\nmin masked: %lf\nmax masked: %lf", min, max);
    contour_depth_F.convertTo(contour_depth_U, CV_8UC1, 255/RANGE_MAX, 0);
    imshow("contour with depth", contour_depth_U);

    float sum = 0;
    int count = 0;
    for(int i = 0; i < contour_depth_F.rows; i++) {
        for(int j = 0; j < contour_depth_F.cols; j++) {

            const float curr = contour_depth_F.at<float>(i, j);
            if (curr == 0 || isnan(curr)) continue;
            //ROS_INFO("curr: %lf", curr);

            sum += curr;
            count++;
        }
    }

    (count == 0) ? (depth = 0) : (depth = sum/count);
    return depth;
}

double calcApproxSurfaceArea(const cv::Mat &contour_depth_F, double depth_average)
{
    double area = 0;

    cv::Mat x_top, x_bottom, x_left, x_right;
    cv::Mat x_bottom_cand, x_right_cand;

    cv::Point top = cv::Point(-1,-1);
    cv::Point left = cv::Point(-1,-1);
    cv::Point bottom, right, bottom_cand, right_cand;

    cv::Point center_cam = cv::Point(contour_depth_F.cols/2, contour_depth_F.rows/2);

    //3d world coordinates that will be reprojected via depth_average
    cv::Point3f top_w, bottom_w, left_w, right_w;

    //find top, bottom, left, and right extreme points
    //find top and bottom
    for (int i = 0; i < contour_depth_F.rows; i++){
        for (int j = 0; j < contour_depth_F.cols; j++){
            const float curr = contour_depth_F.at<float>(i,j);
            if (curr == 0 || isnan(curr)) continue; //TODO why is NaN here???

            if (top.x == -1){

                //homogeneous image coordinates
                //float m[3][1] = {j, i, 1}; //TODO, might be row,col
                //x_top = cv::Mat(3,1,CV_32FC1, m);
                top = cv::Point(j,i); //stores point in (x,y)
                ROS_DEBUG("camera: top at: (%d, %d)", top.x, top.y);
            }

            //will be set to the last non-zero point
            //bottom_cand = cv::Point(i,j);
            //float m[3][1] = {j, i, 1};
            //x_bottom_cand = cv::Mat(3,1,CV_32FC1, m);
            bottom_cand = cv::Point(j,i);
        }
    }
    //x_bottom = x_bottom_cand;
    bottom = bottom_cand;
    ROS_DEBUG("camera: bottom at: (%d, %d)", bottom.x, bottom.y);

    //find left and right
    for (int j = 0; j < contour_depth_F.cols; j++){
        for (int i = 0; i < contour_depth_F.rows; i++){
            const float curr = contour_depth_F.at<float>(i,j);
            if (curr == 0 || isnan(curr)) continue;

            if (left.x == -1){

                //homogeneous image coordinates
                //float m[3][1] = {j, i, 1}; //TODO, might be row,col
                //x_left = cv::Mat(3,1,CV_32FC1, m);
                left = cv::Point(j,i);
                ROS_DEBUG("camera: left at: (%d, %d)", left.x, left.y);
            }

            //will be set to the last non-zero point
            //bottom_cand = cv::Point(i,j);
            //float m[3][1] = {j, i, 1};
            //x_right_cand = cv::Mat(3,1,CV_32FC1, m);
            right_cand = cv::Point(j,i);
        }
    }
    //x_right = x_right_cand;
    right = right_cand;
    ROS_DEBUG("camera: right at: (%d, %d)", right.x, right.y);

    //reproject top, bottom, left, and right using the average depth
    top_w = cv::Point3f((top.x - center_cam.x)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (top.y - center_cam.y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                         depth_average);
    bottom_w = cv::Point3f((bottom.x - center_cam.x)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (bottom.y - center_cam.y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                         depth_average);
    left_w = cv::Point3f((left.x - center_cam.x)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (left.y - center_cam.y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                         depth_average);
    right_w = cv::Point3f((right.x - center_cam.x)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (right.y - center_cam.y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                         depth_average);

    ROS_DEBUG("world: top at: (%lf, %lf)", top_w.x, top_w.y);
    ROS_DEBUG("world: bottom at: (%lf, %lf)", bottom_w.x, bottom_w.y);
    ROS_DEBUG("world: left at: (%lf, %lf)", left_w.x, left_w.y);
    ROS_DEBUG("world: right at: (%lf, %lf)", right_w.x, right_w.y);


    cv::Mat four_points_c = Mat::zeros(contour_depth_F.rows, contour_depth_F.cols, CV_8UC1);
    four_points_c.at<uchar>(top.y,top.x) = 255;
    four_points_c.at<uchar>(bottom.y,bottom.x) = 255;
    four_points_c.at<uchar>(left.y,left.x) = 255;
    four_points_c.at<uchar>(right.y,right.x) = 255;
/*
    cv::Mat four_points_w = Mat::zeros(contour_depth_F.rows, contour_depth_F.cols, CV_8UC1);
    four_points_w.at<uchar>(top_w.y,top_w.x) = 255;
    four_points_w.at<uchar>(bottom_w.y,bottom_w.x) = 255;
    four_points_w.at<uchar>(left_w.y,left_w.x) = 255;
    four_points_w.at<uchar>(right_w.y,right_w.x) = 255;
*/
    cv::dilate(four_points_c, four_points_c, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(-1,-1)), cv::Point(-1,-1), 4);
    //cv::dilate(four_points_w, four_points_w, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(-1,-1)), cv::Point(-1,-1), 4);


    //imshow("camera: contour extreme pixels", four_points_c);
    //imshow("world: contour extreme pixels", four_points_w);

    //calculate the surface area of the plane intersecting the 4 3d points
    area = (bottom_w.y - top_w.y)*(right_w.x - left_w.x);

    return area;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "copter_tracker");
    ros::NodeHandle nh;

    CopterTracker copter_tracker(nh);

    namedWindow("Depth: Edges", WINDOW_AUTOSIZE);
    createTrackbar( "EdgeSliderLow", "Depth: Edges", &copter_tracker.edge_thresh_low, EDGE_LOW_MAX);
    createTrackbar( "EdgeSliderHigh", "Depth: Edges", &copter_tracker.edge_thresh_high, EDGE_HIGH_MAX);

    namedWindow("Depth: Cleaned-Up", WINDOW_AUTOSIZE);
    createTrackbar( "DepthScaleFactor", "Depth: Cleaned-Up", &copter_tracker.depth_scale_factor, DEPTH_SCALE_FACTOR_MAX);
    createTrackbar( "copyOut(0), interpolate(1)", "Depth: Cleaned-Up", &copter_tracker.function_toggle, NUM_FUNCTIONS-1);
    createTrackbar( "Blur Size", "Depth: Cleaned-Up", &copter_tracker.blur_size, BLUR_SIZE_MAX);


//    ros::ServiceServer switch_service = nh.advertiseService("model_switch",
//            &MotionDetector::switch_callback, &motion_detector);
    ros::spin();

    return 0;
}
