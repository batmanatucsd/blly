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

#define CROP_WIDTH 320
#define CROP_HEIGHT 240

#define THRESH_MAG 3

#define RANGE_MAX 9.757 //TODO find real max based on range of sensor (mm or meters)
#define EDGE_LOW_MAX 100
#define EDGE_HIGH_MAX 100

#define DEPTH_SCALE_FACTOR_MAX 100
#define NUM_FUNCTIONS 2
#define BLUR_SIZE_MAX 100

using namespace std;
using namespace cv;


// Forward Declaration
static void drawMotionIntensity(const cv::Mat&, cv::Mat&);
static void drawOptFlowMap(const cv::Mat&, cv::Mat&,
        int, double, const cv::Scalar&);
static void drawRectFromContours(cv::Mat&,
        std::vector<std::vector<cv::Point> >&);
//get ride of erroneous black values
//these happen by noise or when there is no surface reflecting back kinect IR
static void threshBlack(const cv::Mat&, cv::Mat&);

static void copyOutBlack(const cv::Mat&, cv::Mat&, float);
static void interpOutBlack(const cv::Mat&, cv::Mat&, float);
static void interp1D(cv::Mat&, int, int, int, int, int, int);


class CopterTracker
{
    image_transport::ImageTransport iTrans;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;
    std::vector<std::vector<cv::Point> > contours;


    void callback_depth(const sensor_msgs::ImageConstPtr& msg);

    public:
    int edge_thresh_low;
    int edge_thresh_high;
    int depth_scale_factor;
    int function_toggle;
    int blur_size;

    CopterTracker(ros::NodeHandle n): iTrans(n)
    {
        image_sub = iTrans.subscribe("/camera/depth/image", 1, &CopterTracker::callback_depth, this);
        image_pub = iTrans.advertise("/camera/depth_cleaned/image", 1);

        edge_thresh_low = 8;
        edge_thresh_high = 15;
        depth_scale_factor = 100/8;
        function_toggle = 0;
        blur_size = 7;
        //algorithm_mode = FOFA; //intializes the algorithm to FOFA
    }



};

void CopterTracker::callback_depth(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr frame_original_ptr;

//    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
//            cv::Size(3,3), cv::Point(-1,-1));
//    std::string label;

    try {
        frame_original_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //convert to grayscale and make a copy
    cv::Mat depth_img;
    frame_original_ptr->image.convertTo(depth_img, CV_8UC1, 255/RANGE_MAX, 0);//scale by the max value and shift by 0



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

    Mat depth_img_edges;
    Canny(depth_img_cleaned, depth_img_edges, edge_thresh_low, edge_thresh_high);

    cv::waitKey(1);
    imshow("depth", depth_img);
    imshow("Depth: Cleaned-Up", depth_img_cleaned);
    imshow("Depth: Edges", depth_img_edges);

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

static void drawRectFromContours(cv::Mat& frame, std::vector<std::vector<cv::Point> > &contours)
{
    if (contours.empty()) { return; }
    //only draw rectangle on first contour
    std::vector<cv::Point> contour = contours.front();
    int maxSize = 0;

    //find the largest contour
    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin() ; it != contours.end(); ++it){
        if (it->size() > maxSize){
            contour = *it;
            maxSize = it->size();
        }
    }

    int minX = INT_MAX;
    int maxX = 0;
    int minY = INT_MAX;
    int maxY = 0;

    cv::Point point;
    for (int i = 0; i < contour.size(); i++){
        point = contour.at(i);

        if (point.x < minX){
            minX = point.x;
        }
        if (point.x > maxX){
            maxX = point.x;
        }
        if (point.y < minY){
            minY = point.y;
        }
        if (point.y > maxY){
            maxY = point.y;
        }
    }

    cv::rectangle(frame, cv::Point(minX, minY), cv::Point(maxX,maxY), cv::Scalar(0,0,255), 5);
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
