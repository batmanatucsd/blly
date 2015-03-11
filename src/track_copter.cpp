#include "ros/ros.h"
#include "blly/use_classifier.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <cmath>
#include <vector>

#include <boost/thread/mutex.hpp>
#include <string>

#define RGB_WIDTH 640
#define RGB_HEIGHT 480

#define THRESH_MAG 3

#define RANGE_MAX 9.757 //TODO find real max and min based on range of sensor (mm or meters) might be erronious noise max
#define RANGE_MIN 0.01
#define SLICE_DEPTH 1.0

#define DEPTH_OF_FOCUS_MAX 100
#define EDGE_LOW_MAX 100
#define EDGE_HIGH_MAX 100

#define DEPTH_SCALE_FACTOR_MAX 100
#define NUM_FUNCTIONS 2
#define BLUR_SIZE_MAX 100
//if 1, contours must be square. smaller value allows for skinnier contours
#define CONTOUR_ECCENTRICTY_THRESH 0.45 //(1 is perfect square)TODO look into the value for best approx
#define IR_CAMERA_FOCAL_PX 575.8 //(focal point in pixels) //taken from wiki.ros.org/kinect_calibration/technical
#define IR_CAMERA_FOCAL_PY 575.8
#define IR_CAMERA_CENTER_X 314.5
#define IR_CAMERA_CENTER_Y 235.5

#define SCALE_AREA_MIN 0.0003
#define SCALE_AREA_MAX 0.03

#define CASCADE_WIDTH 100 //pixel width and height for cascade classifier. images will be cropped to this size
#define CASCADE_HEIGHT 40
#define TRAIN_CASCADE 0 //false
#define TRAIN_PATH_NEG "//home/frank/turtlebot/src/blly/src/neg/"
#define TRAIN_PATH_POS "//home/frank/turtlebot/src/blly/src/pos/"
#define COPTER_CLASSIFIER_PATH "//home/frank/turtlebot/src/blly/src/classifiers/copter_all_500_500/cascade.xml"

using namespace std;
using namespace cv;


// Forward Declaration
static void drawMotionIntensity(const cv::Mat&, cv::Mat&);
static void drawOptFlowMap(const cv::Mat&, cv::Mat&, int, double, const cv::Scalar&);
static void drawRectsFromContours(cv::Mat&, std::vector<std::vector<cv::Point> >&);
static void getCroppedImagesFromContours(cv::Mat &, std::vector<cv::Mat> &, std::vector<std::vector<cv::Point> > &);

static void getDepthOfFocus(const cv::Mat&, cv::Mat&, double, double);


static void viewImages(std::vector<cv::Mat> &, string, int&, int&, bool&);
static void viewImages(std::vector<cv::Mat> &, string);


static void filterContoursByShape(std::vector<std::vector<cv::Point> > &, std::vector<std::vector<cv::Point> > &, float);
static void filterContoursByScale(std::vector<std::vector<cv::Point> > &, std::vector<cv::Point3f> &, std::vector<std::vector<cv::Point> > &, const cv::Mat&, const cv::Mat&);
cv::Point3f filterByAppearance(std::vector<cv::Mat> &, std::vector<cv::Point3f> &, std::vector<cv::Mat> &, cv::CascadeClassifier &);// Ptr<FeatureEvaluator>&

double findContourDepth(const cv::Mat &);
double calcApproxSurfaceArea(const cv::Mat &, double, cv::Point3f &);

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

    bool use_classifier;

    image_transport::ImageTransport iTrans;

    //image_transport::Publisher image_pub;
    ros::Publisher copter_center_3d_pub;
    ros::Publisher copter_center_2d_pub;
    ros::Publisher copter_center_3d_stamped_pub;

    image_transport::Subscriber image_sub_depth;
    image_transport::Subscriber image_sub_rgb;
    image_transport::Publisher image_pub;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > contours_cand;
    std::vector<std::vector<cv::Point> > contours_finalists;
    std::vector<std::vector<cv::Point> > shared_contours_finalists;
    std::vector<std::vector<cv::Point> > contours_finalists_rgb;

    std::vector<cv::Point3f> centers_3d_finalists;
    std::vector<cv::Point3f> shared_centers_3d_finalists;
    std::vector<cv::Point3f> centers_3d_finalists_rgb;


    std::vector<cv::Mat> cropped_finalists;
    std::vector<cv::Mat> copters;

    blly::use_classifier srv;
    ros::ServiceServer service;

    int numPos, numNeg;

    void callback_depth(const sensor_msgs::ImageConstPtr& msg);
    void callback_rgb(const sensor_msgs::ImageConstPtr& msg);


    public:
    int edge_thresh_low;
    int edge_thresh_high;
    int depth_scale_factor;
    int function_toggle;
    int blur_size;
    bool training_cascade;
    int depth_of_focus;

    cv::CascadeClassifier copter_classifier;
    Ptr<FeatureEvaluator> feval;

    cv::Mat kernel;

    CopterTracker(ros::NodeHandle n): iTrans(n)
    {
        //TODO possibly subscribe to rectified depth to improve reprojection accuracy
        image_sub_depth = iTrans.subscribe("/camera/depth/image", 1, &CopterTracker::callback_depth, this);
        image_sub_rgb = iTrans.subscribe("/camera/rgb/image_color", 1, &CopterTracker::callback_rgb, this);
        image_pub = iTrans.advertise("/camera/rgb/copter_boxed", 1);

        //image_pub = iTrans.advertise("/camera/depth_cleaned/image", 1);
        copter_center_3d_pub = n.advertise<geometry_msgs::Point>("copter_center_3d", 1);
        copter_center_3d_stamped_pub = n.advertise<geometry_msgs::PointStamped>("copter_center_stamped_3d", 1);
        copter_center_2d_pub = n.advertise<geometry_msgs::Point>("copter_center_2d", 1);

        service = n.advertiseService("use_classifier", &CopterTracker::toggle_classifier_callback, this);

        edge_thresh_low = 32;
        edge_thresh_high = 46;
        depth_scale_factor = 100/8;
        function_toggle = 0;
        blur_size = 7;
        depth_of_focus = 10;
        kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(-1,-1));
        rgb_locked_writing = false;
        rgb_locked_reading = false;
        //shared_contours_finalists_writing = false;
        //shared_contours_finalists_reading = false;
        use_classifier = true;

        numPos = 0;
        numNeg = 0;
        training_cascade = TRAIN_CASCADE;

        if( !copter_classifier.load(COPTER_CLASSIFIER_PATH) ){
            ROS_INFO("--(!)Error loading face cascade");
        }
        feval = FeatureEvaluator::create(1);//TODO might be 1 or 2. enum HAAR or LBP

        //algorithm_mode = FOFA; //intializes the algorithm to FOFA
    }

    bool toggle_classifier_callback(blly::use_classifier::Request &req, blly::use_classifier::Response &res)
    {
        switch (req.use_classifier)
        {
            case 1:
                ROS_INFO("requested to use Cascade Classifier");
                use_classifier = true;
                break;
            case 0:
                ROS_INFO("requested to NOT use Cascade Classifier");
                use_classifier = false;
                break;
            default: break;
        }
        return true;
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
    //return;

    double min;
    double max;
    minMaxLoc(depth_orig->image, &min, &max);
    //ROS_INFO("\nmin: %lf\nmax: %lf", min, max);

    //TODO take advantage of decimal pixel data before converting to uint8
    //take depth frames centered at some depth d, with width w. then convert those frames to uint8
    //sliding depth frme (focus on a depth range) and use histEQ on each slice

    //TODO investigate a tracking algorithm to speed up detection and searching

    cv::Mat depth_median;
    medianBlur(depth_orig->image, depth_median, 5);

    //convert to grayscale and make a copy
    cv::Mat depth_img;
    //depth_orig->image.convertTo(depth_img, CV_8UC1, 255/RANGE_MAX, 0);//scale by the max value and shift by 0
    depth_orig->image.convertTo(depth_img, CV_8UC1, 255/RANGE_MAX, 0);//scale by the max value and shift by 0

    cv::Rect center_rect(200,200,100,100);
    //imshow("center table", depth_img(center_rect));
    cv::waitKey(1);

    cv::Mat table_img = depth_median(center_rect);
    cv:: Scalar mean_scalar = mean(table_img);
    //ROS_INFO("average depth: %lf", mean_scalar[0]);

    minMaxLoc(depth_median, &min, &max);
    //ROS_INFO("\nmin: %lf\nmax: %lf", min, max);

    //return;


    cv::Mat depth_img_dof;
    double center = RANGE_MIN + SLICE_DEPTH/2 + double(depth_of_focus)/DEPTH_OF_FOCUS_MAX*(RANGE_MAX - (RANGE_MIN + SLICE_DEPTH));
    getDepthOfFocus(depth_orig->image, depth_img_dof, center, SLICE_DEPTH);

    //imshow("Depth of Focus", depth_img_dof);
    cv::waitKey(1);

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

    imshow("depth", depth_img);
    imshow("Depth: Cleaned-Up", depth_img_cleaned);
    imshow("Depth: Edges", depth_img_edges);

    imshow("Candidate Contours: filtered by shape", contour_img);

    //Mat test_img = Mat::zeros(depth_img_edges.rows, depth_img_edges.cols, CV_8UC1);
    //drawContours(test_img, contours_cand, -1, 255, CV_FILLED);
    //imshow("test", test_img);

    contours_finalists.clear();
    centers_3d_finalists.clear();
    //filter based on scale (should be rough size of copter)
    filterContoursByScale(contours_finalists, centers_3d_finalists, contours_cand, depth_orig->image, kernel);

    Mat contour_img_finalists = Mat::zeros(depth_img_edges.rows, depth_img_edges.cols, CV_8UC1);
    drawRectsFromContours(contour_img_finalists, contours_finalists);

    imshow("contour finalists: filtered by shape and scale", contour_img_finalists);
    //for (std::vector<cv::Point3f>::iterator it = centers_3d_finalists.begin() ; it != centers_3d_finalists.end(); ++it){
    //    cv::Point3f p = *it;
        //ROS_INFO("center finalist: (%lf, %lf, %lf)", p.x, p.y, p.z);
    //}


    if (!shared_contours_finalists_reading) {
        shared_contours_finalists_writing = true;
        shared_contours_finalists = contours_finalists;
        shared_centers_3d_finalists = centers_3d_finalists;
        shared_contours_finalists_writing = false;
    }

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

    //TODO test if bluring yeilds better results
    cv::GaussianBlur(gray_img, gray_img, cv::Size(3,3), 3, cv::BORDER_DEFAULT);

    //TODO mutex lock to make this if statement obsolete
    if (!shared_contours_finalists_writing){

        shared_contours_finalists_reading = true;
        contours_finalists_rgb = shared_contours_finalists;
        centers_3d_finalists_rgb = shared_centers_3d_finalists;
        shared_contours_finalists_reading = false;

        //check to make sure it is not accessed before populated
        //if (contours_finalists_rgb.empty()) return;

        //Mat contour_img_finalists = Mat::zeros(gray_img.rows, gray_img.cols, CV_8UC1);
        //drawRectsFromContours(contour_img_finalists, contours_finalists_rgb);
        //drawRectsFromContours(gray_img, contours_finalists_rgb);
        getCroppedImagesFromContours(gray_img, cropped_finalists, contours_finalists_rgb);

        viewImages(cropped_finalists, string("cropped Finalists"), numPos, numNeg, training_cascade);

        //croppedFinalists.clear();//testing entire image
        //croppedFinalists.push_back(gray_img);
        if (!training_cascade && use_classifier){
            //send croppedFinalists to cascade classifier for final filtering
            cv::Point3f copter_center_3d = filterByAppearance(cropped_finalists, centers_3d_finalists_rgb, copters, copter_classifier);//feval
            viewImages(copters, string("copters"));
            if (copter_center_3d.x != 0 || copter_center_3d.y != 0 || copter_center_3d.z != 0){
                //ROS_INFO("Copter found at: (%lf, %lf, %lf)", copter_center_3d.x, copter_center_3d.y, copter_center_3d.z);
                //TODO publish the location of the copter. also publish it in image pixels by projecting to image plane
                cv::Point copter_center_2d = cv::Point(copter_center_3d.x/copter_center_3d.z*IR_CAMERA_FOCAL_PX + IR_CAMERA_CENTER_X, copter_center_3d.y/copter_center_3d.z*IR_CAMERA_FOCAL_PY + IR_CAMERA_CENTER_Y);
                //ROS_INFO("Pixel location: (%d, %d)", copter_center_2d.x, copter_center_2d.y);

                geometry_msgs::Point msg_3d;
                msg_3d.x = copter_center_3d.x;
                msg_3d.y = copter_center_3d.y;
                msg_3d.z = copter_center_3d.z;

                copter_center_3d_pub.publish(msg_3d);

                geometry_msgs::PointStamped msg_3d_s;
                msg_3d_s.header.frame_id = "static_camera_frame";
                msg_3d_s.point = msg_3d;
                copter_center_3d_stamped_pub.publish(msg_3d_s);

                geometry_msgs::Point msg_2d;
                msg_2d.x = copter_center_2d.x;
                msg_2d.y = copter_center_2d.y;
                msg_2d.z = 0;
                copter_center_2d_pub.publish(msg_2d);

                cv::Mat copter_img = copters[0];
                cv::Rect box(copter_center_2d.x - copter_img.cols/2, copter_center_2d.y - copter_img.rows/2, copter_img.cols, copter_img.rows);
                cv::rectangle(rgb_orig->image, box, cv::Scalar(0,0,255), 4);

            }

        }

        if (!training_cascade && !use_classifier){

            int count = 0;
            for (std::vector<cv::Point3f> ::iterator it = centers_3d_finalists_rgb.begin() ; it != centers_3d_finalists_rgb.end(); ++it){

                cv::Point3f copter_center_3d = *it;

                if (copter_center_3d.x != 0 || copter_center_3d.y != 0 || copter_center_3d.z != 0){
                    //ROS_INFO("Copter found at: (%lf, %lf, %lf)", copter_center_3d.x, copter_center_3d.y, copter_center_3d.z);
                    //TODO publish the location of the copter. also publish it in image pixels by projecting to image plane
                    cv::Point copter_center_2d = cv::Point(copter_center_3d.x/copter_center_3d.z*IR_CAMERA_FOCAL_PX + IR_CAMERA_CENTER_X, copter_center_3d.y/copter_center_3d.z*IR_CAMERA_FOCAL_PY + IR_CAMERA_CENTER_Y);
                    //ROS_INFO("Pixel location: (%d, %d)", copter_center_2d.x, copter_center_2d.y);

                    geometry_msgs::Point msg_3d;
                    msg_3d.x = copter_center_3d.x;
                    msg_3d.y = copter_center_3d.y;
                    msg_3d.z = copter_center_3d.z;

                    copter_center_3d_pub.publish(msg_3d);

                    geometry_msgs::PointStamped msg_3d_s;
                    msg_3d_s.header.frame_id = "static_camera_frame";
                    msg_3d_s.point = msg_3d;
                    copter_center_3d_stamped_pub.publish(msg_3d_s);

                    geometry_msgs::Point msg_2d;
                    msg_2d.x = copter_center_2d.x;
                    msg_2d.y = copter_center_2d.y;
                    msg_2d.z = 0;
                    copter_center_2d_pub.publish(msg_2d);

                    cv::Rect box = boundingRect(contours_finalists_rgb[count]);
                    cv::rectangle(rgb_orig->image, box, cv::Scalar(0,0,255), 4);
                    //image_pub.publish(rgb_orig->toImageMsg());

                    count++;
                }
            }

        }

        image_pub.publish(rgb_orig->toImageMsg());


    }

    imshow("gray image", gray_img);
    cropped_finalists.clear();
    copters.clear();
    centers_3d_finalists_rgb.clear();

}

static void getDepthOfFocus(const cv::Mat& depth_orig, cv::Mat& depth_dof, double center, double width){

    cv::Mat depth_dof_32 = Mat::zeros(depth_orig.rows, depth_orig.cols, CV_32FC1);

    double thresh_min = center - width/2;
    double thresh_max = center + width/2;
    float curr;
    for (int i = 0; i < depth_orig.rows; i++){
        for (int j = 0; j < depth_orig.cols; j++){
            curr = depth_orig.at<float>(i,j);

            if (curr > thresh_min && curr < thresh_max){
                depth_dof_32.at<float>(i,j) = curr;
            }

        }

    }

    //depth_dof = depth_dof_32.clone();

    double min;
    double max;
    minMaxLoc(depth_dof_32, &min, &max);
    //shift to zero
    depth_dof_32 = depth_dof_32 - min;

    cv::Mat temp;
    if (max == 0){
        depth_dof = Mat::zeros(depth_orig.rows, depth_orig.cols, CV_8UC1);
        return;
    }
    depth_dof_32.convertTo(temp, CV_8UC1, 255/(max-min));
    depth_dof = temp.clone();


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

static void getCroppedImagesFromContours(cv::Mat &ipImage, std::vector<cv::Mat> &opImages, std::vector<std::vector<cv::Point> > &contours){

    if (contours.empty()) { return; }

    float width_adjusted, bounding_rect_center_x, right_bound, left_bound;
    cv::Rect rect_adjusted;
    cv::Mat imageCropped;

    //crop all contours as images of the same size
    //TODO maybe determine copter orientation: ie: if viewing front, back, left or right sides of copter and crop accordingly
    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin() ; it != contours.end(); ++it){
        cv::Rect bounding_rect = boundingRect(*it);

        //calculate width such that ratio is maintained
        width_adjusted = bounding_rect.height*float(CASCADE_WIDTH)/CASCADE_HEIGHT;

        //case where copter is really close to camera
        if  (width_adjusted > RGB_WIDTH) continue;

        bounding_rect_center_x = bounding_rect.x + float(bounding_rect.width)/2;
        right_bound = bounding_rect_center_x + float(width_adjusted)/2 ;
        left_bound = bounding_rect_center_x - float(width_adjusted)/2;

        //create an adjusted rectangle with the new width and height from bounding_rect
        //make sure new rect doesn't go out of bounds of the image. shift appropriately
        if (right_bound > RGB_WIDTH){

            rect_adjusted = cv::Rect(left_bound - (right_bound - RGB_WIDTH), bounding_rect.y, width_adjusted, bounding_rect.height);

        }
        else if (left_bound < 0){

            rect_adjusted = cv::Rect(left_bound - left_bound, bounding_rect.y, width_adjusted, bounding_rect.height);
        }
        else {
            rect_adjusted = cv::Rect(left_bound, bounding_rect.y, width_adjusted, bounding_rect.height);
        }


        //crops image
        imageCropped = ipImage(rect_adjusted);

        //resize
        cv::resize(imageCropped, imageCropped, cv::Size(CASCADE_WIDTH, CASCADE_HEIGHT));

        opImages.push_back(imageCropped);

    }

}

static void filterContoursByScale(std::vector<std::vector<cv::Point> > &contours_finalists, std::vector<cv::Point3f> &centers_3d,
                                  std::vector<std::vector<cv::Point> > &contours_cand, const cv::Mat& depth_img_orig, const cv::Mat& kernel)
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

        //increase the contour back to normal size //TODO test this now
        cv::dilate(contour_mask, contour_mask, kernel, cv::Point(-1,-1), 4);
        imshow("contour mask 2", contour_mask*255);
        contour_mask.convertTo(contour_mask_F, CV_32FC1);
        contour_depth_F = depth_img_orig.mul(contour_mask_F);


        //filter based on scale, not just depth
        if (depth_average > 0){

            cv::Point3f center_3d;
            //TODO pass in larger contour by 4
            scale_area = calcApproxSurfaceArea(contour_depth_F, depth_average, center_3d);

            //ROS_INFO("depth: %.6f", depth_average);
            //ROS_INFO("area: %.6f\n", scale_area);

            if (scale_area >= SCALE_AREA_MIN && scale_area <= SCALE_AREA_MAX){
                contours_finalists.push_back(*it);
                centers_3d.push_back(center_3d);
                //ROS_INFO("valid area: %.6f\n", scale_area);
/*
                //TODO test that area min and max are set well
                cv::Mat depth_img;
                depth_img_orig.convertTo(depth_img, CV_8UC1, 255/RANGE_MAX, 0);
                cv::Rect rect = boundingRect(*it);
                imshow("check for valid area", depth_img(rect));
                cv::rectangle(depth_img, rect, cv::Scalar(0,0,255), 3);
                imshow("check for valid area2", depth_img);
                cv::waitKey(0);
*/
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

cv::Point3f filterByAppearance(std::vector<cv::Mat> &croppedFinalists, std::vector<cv::Point3f> &centers_3d, std::vector<cv::Mat> &copters, cv::CascadeClassifier &copter_classifier){ //, Ptr<FeatureEvaluator>& feval

    if (croppedFinalists.empty()) { return cv::Point3f(0,0,0); }

    std::vector<cv::Rect> results;
    cv::Mat copter_img;
    int count = 0;
    cv::Point3f center_3d = cv::Point3f(0,0,0); //if nothing is found, returns (0,0,0)
    //run all the finalists through the classifier
    for (std::vector<cv::Mat>::iterator it = croppedFinalists.begin() ; it != croppedFinalists.end(); ++it){

        //Ptr<FeatureEvaluator> feat_eval = FeatureEvaluator::create(1);
        //copter_classifier.setImage(feat_eval, *it);
        //result = copter_classifier.runAt(feat_eval, cv::Point(0,0));


        //TODO: problem. multiple results are being recorded. yet there is only one copter in the image
        copter_classifier.detectMultiScale(*it, results, 1.1);

        if (!results.empty()){
            for (std::vector<cv::Rect>::iterator it_results = results.begin() ; it_results != results.end(); ++it_results){
                //add the image cropped at the rectangle
                copter_img = *it;
                copters.push_back(copter_img(*it_results));
                center_3d = centers_3d.at(count); //centers_3d holds parallel values of center_3d for each finalist
                ROS_INFO("copter found!");

            }
            results.clear();
        }
        else {
            //ROS_INFO("candidate rejected! no results found");
            //imshow("rejected candidate", *it);
            //cv::waitKey(0);
        }

        count++;
    }

    return center_3d;

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

double calcApproxSurfaceArea(const cv::Mat &contour_depth_F, double depth_average, cv::Point3f &center_3d)
{
    double area = 0;

    cv::Mat x_top, x_bottom, x_left, x_right;
    cv::Mat x_bottom_cand, x_right_cand;

    cv::Point top = cv::Point(-1,-1);
    cv::Point left = cv::Point(-1,-1);
    cv::Point bottom, right, bottom_cand, right_cand;

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
    top_w = cv::Point3f((top.x - IR_CAMERA_CENTER_X)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (top.y - IR_CAMERA_CENTER_Y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                         depth_average);
    bottom_w = cv::Point3f((bottom.x - IR_CAMERA_CENTER_X)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (bottom.y - IR_CAMERA_CENTER_Y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                         depth_average);
    left_w = cv::Point3f((left.x - IR_CAMERA_CENTER_X)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (left.y - IR_CAMERA_CENTER_Y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                         depth_average);
    right_w = cv::Point3f((right.x - IR_CAMERA_CENTER_X)/float(IR_CAMERA_FOCAL_PX)*depth_average,
                        (right.y - IR_CAMERA_CENTER_Y)/float(IR_CAMERA_FOCAL_PX)*depth_average,
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
    //dilate so points are visible
    cv::dilate(four_points_c, four_points_c, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(-1,-1)), cv::Point(-1,-1), 4);
    //cv::dilate(four_points_w, four_points_w, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(-1,-1)), cv::Point(-1,-1), 4);


    imshow("camera: contour extreme pixels", four_points_c);
    //imshow("world: contour extreme pixels", four_points_w);

    //calculate the surface area of the plane intersecting the 4 3d points
    area = (bottom_w.y - top_w.y)*(right_w.x - left_w.x);
    center_3d = cv::Point3f(left_w.x + (right_w.x - left_w.x)/2, top_w.y + (bottom_w.y - top_w.y)/2, depth_average);

    return area;
}

static void viewImages(std::vector<cv::Mat> & images, string window_name, int &numPos, int &numNeg, bool &training_cascade){

    if (images.empty()) { return; }

    int key;

    //crop all contours as images of the same size
    //TODO maybe determine copter orientation: ie: if viewing front, back, left or right sides of copter and crop accordingly
    for (std::vector<cv::Mat>::iterator it = images.begin() ; it != images.end(); ++it){

        imshow(window_name, *it);
        char buffer[10];

        if (training_cascade){
            //FOR TRAINING CASCADE CLASSIFIER
            std::string filename;
            key = cv::waitKey(0);
            //if positive, save as positive
            if (key == 112){
                numPos++;
                sprintf(buffer, "%d", numPos);
                filename = string(TRAIN_PATH_POS) + string(buffer) + string(".pgm");
                ROS_INFO("numPos: %d", numPos);
                try {
                        cv::imwrite(filename, *it);
                    }
                    catch (runtime_error& ex) {
                        ROS_INFO("Exception converting image to format: %s", ex.what());

                    }
                /*
                if (cv::imwrite(filename, *it))
                {
                    ROS_INFO("saved positive image number: %d", numPos);
                }
                else {
                    ROS_INFO("error saving image");
                }*/

            }
            //if negative, save as negative
            else if (key == 110){
                numNeg++;
                sprintf(buffer, "%d", numNeg);
                filename = string(TRAIN_PATH_NEG) + string(buffer) + string(".pgm");
                ROS_INFO("numNeg: %d", numNeg);
                try {
                        cv::imwrite(filename, *it);
                    }
                    catch (runtime_error& ex) {
                        ROS_INFO("Exception converting image to format: %s", ex.what());

                    }
                /*
                if (cv::imwrite(filename, *it))
                {
                    ROS_INFO("saved negative image number: %d", numNeg);
                }
                else {
                    ROS_INFO("error saving image");
                }*/
            }
        }
    }

}
static void viewImages(std::vector<cv::Mat> & images, string window_name){

    if (images.empty()) { return; }

    //crop all contours as images of the same size
    //TODO maybe determine copter orientation: ie: if viewing front, back, left or right sides of copter and crop accordingly
    for (std::vector<cv::Mat>::iterator it = images.begin() ; it != images.end(); ++it){

        imshow(window_name, *it);
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "copter_tracker");
    ros::NodeHandle nh;

    CopterTracker copter_tracker(nh);
    if (argc > 1){
        if (string(argv[1]) == "true"){
            copter_tracker.training_cascade = true;
        }
    }
    namedWindow("Depth: Edges", WINDOW_AUTOSIZE);
    createTrackbar( "EdgeSliderLow", "Depth: Edges", &copter_tracker.edge_thresh_low, EDGE_LOW_MAX);
    createTrackbar( "EdgeSliderHigh", "Depth: Edges", &copter_tracker.edge_thresh_high, EDGE_HIGH_MAX);

    namedWindow("Depth: Cleaned-Up", WINDOW_AUTOSIZE);
    createTrackbar( "DepthScaleFactor", "Depth: Cleaned-Up", &copter_tracker.depth_scale_factor, DEPTH_SCALE_FACTOR_MAX);
    createTrackbar( "copyOut(0), interpolate(1)", "Depth: Cleaned-Up", &copter_tracker.function_toggle, NUM_FUNCTIONS-1);
    createTrackbar( "Blur Size", "Depth: Cleaned-Up", &copter_tracker.blur_size, BLUR_SIZE_MAX);

    namedWindow("Depth of Focus", WINDOW_AUTOSIZE);
    createTrackbar( "Depth of focus:", "Depth of Focus", &copter_tracker.depth_of_focus, DEPTH_OF_FOCUS_MAX);


//    ros::ServiceServer switch_service = nh.advertiseService("model_switch",
//            &MotionDetector::switch_callback, &motion_detector);
    ros::spin();

    return 0;
}
