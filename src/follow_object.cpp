#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

class FollowObject {
public:
  ros::NodeHandle handler;
  ros::Publisher vel_pub;
  ros::Subscriber point_sub;
  geometry_msgs::Twist vel;
  bool stop; 

  FollowObject();

private:
  void Callback(const geometry_msgs::Point::ConstPtr&);
  double linear_scale, angular_scale;
  double goal_z;
};

FollowObject::FollowObject() : goal_z(0.5), linear_scale(0.25), angular_scale(1.5), stop(true) {
  handler.param("linear_scale", linear_scale, linear_scale);
  handler.param("angular_linear", angular_scale, angular_scale);

  vel_pub = handler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity" , 1, true);
  point_sub = handler.subscribe<geometry_msgs::Point>("/copter_center_3d", 10,
                  &FollowObject::Callback, this);

  vel.linear.x = 0;
  vel.angular.z = 0;
}

void FollowObject::Callback(const geometry_msgs::Point::ConstPtr& point) {
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

double findContourDepth(const cv::Mat &contour_depth_F) {
    double depth;
    double min;
    double max;
    minMaxLoc(contour_depth_F, &min, &max);
    cv::Mat contour_depth_U(contour_depth_F.rows, contour_depth_F.cols, CV_8UC1);
    //ROS_INFO("\nmin masked: %lf\nmax masked: %lf", min, max);
    contour_depth_F.convertTo(contour_depth_U, CV_8UC1, 255/RANGE_MAX, 0);
    //imshow("contour with depth", contour_depth_U);

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
