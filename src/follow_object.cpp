#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

class FollowObject {
public:
  FollowObject();
  Callback();

private:
  double linear_scale, angular_scale;
  ros:NodeHandle handler;
  ros::Publisher vel_pub;
  ros::Subscriber point_sub;
};

FollowObject::FollowObject() {

  handler.param("linear_scale", linear_scale, linear_scale);
  handler.param("angular_linear", angular_scale, angular_scale);

  vel_pub = handler.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  point_sub = handler.subscribe<geometry_msgs::Point>("frank", 10,
                  &FollowObject::Callback, this);

}

FollowObject::Callback(const geometry_msgs::Point::ConstPtr& point) {
  geometry_msgs::Twist vel;
  vel.angular.z = angular_scale*point->x;
  vel.linear.x = linear_scale*point->z
  vel_pub.publish(vel);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_object");
  FollowObject follow_object;
  //ros::spin();
  geometry_msgs::Twist vel;
  vel.linear.x = linear_scale; 
  vel_pub.publish(vel);
}
