#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

double findAverageDepth(const cv::Mat &);

class FollowObject {
public:
  ros::Publisher vel_pub;
  geometry_msgs::Twist vel;
  bool stop; 

  FollowObject();

private:
  void callback(const geometry_msgs::Point::ConstPtr&);
  void callback_depth(const sensor_msgs::ImageConstPtr& msg);

  double linear_scale, angular_scale;
  double goal_z;

  ros::NodeHandle handler;
  image_transport::ImageTransport iTrans;
  image_transport::Subscriber image_sub_depth;
  ros::Subscriber point_sub;
};

FollowObject::FollowObject() : goal_z(0.5), linear_scale(0.25), angular_scale(1.5), stop(true), iTrans(handler) {
  handler.param("linear_scale", linear_scale, linear_scale);
  handler.param("angular_linear", angular_scale, angular_scale);

  vel_pub = handler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity" , 1, true);
  point_sub = handler.subscribe<geometry_msgs::Point>("/copter_center_3d", 10,
                  &FollowObject::callback, this);
  image_sub_depth = iTrans.subscribe("/camera/depth/image", 1, &FollowObject::callback_depth, this);

  vel.linear.x = 0;
  vel.angular.z = 0;
}

void FollowObject::callback(const geometry_msgs::Point::ConstPtr& point) {
  if(point->x != 0 && point->y != 0 && point->z > 0) {
    vel.angular.z = -angular_scale*point->x;
    vel.linear.x = linear_scale*(point->z - goal_z);
    if(point->z - goal_z < 0)
        stop = true;
    else
        stop = false;

  } else {
    vel.linear.x = 0;
  }

  vel_pub.publish(vel);
}

void FollowObject::callback_depth(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr depth_orig;
   
  try {
      depth_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  double average_depth = findAverageDepth(depth_orig->image);
  
  ROS_INFO("AVERAGE is DEPTH %lf", average_depth);

  if(average_depth < 1.1) {
    vel.linear.x = 0; 
    vel.angular.z = 0.5;
    vel_pub.publish(vel);
  }
}

double findAverageDepth(const cv::Mat &depth_image_F) {
    double depth = 0, sum = 0;
    int count = 0;

    for(int i = 0; i < depth_image_F.rows; i++) {
        for(int j = 0; j < depth_image_F.cols; j++) {

            const float curr = depth_image_F.at<float>(i, j);
            if (curr == 0 || isnan(curr)) continue;

            sum += curr;
            count++;
        }
    }

    (count == 0) ? (depth = 0) : (depth = sum/count);
    return depth;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "follow_object");
  FollowObject follow_object;

  if(follow_object.stop)
      follow_object.vel.linear.x = 0;
   
  while(ros::ok()) {
    follow_object.vel_pub.publish(follow_object.vel);
    ros::spinOnce();
  }
}
