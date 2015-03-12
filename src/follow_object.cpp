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

FollowObject::FollowObject() : goal_z(0.6), linear_scale(1.0), angular_scale(1.5), stop(false) {
  handler.param("linear_scale", linear_scale, linear_scale);
  handler.param("angular_linear", angular_scale, angular_scale);

  vel_pub = handler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity" , 1, true);
  point_sub = handler.subscribe<geometry_msgs::Point>("/copter_center_3d", 10,
                  &FollowObject::Callback, this);

}

void FollowObject::Callback(const geometry_msgs::Point::ConstPtr& point) {
  if(point->z > 0) {
    vel.angular.z = angular_scale*point->x;
    vel.linear.x = linear_scale*(point->z - goal_z);
    if(point->z - goal_z < 0)
      stop = true;

  } else {
    vel.linear.x = 0;
  }

  vel_pub.publish(vel);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "follow_object");
  FollowObject follow_object;
  //ros::spin();
  
  // DEBUG
  //ros::NodeHandle handler;
  //ros::Publisher vel_pub;
  //vel_pub = handler.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  while(ros::ok()) {
    if(!follow_object.stop)
      follow_object.vel_pub.publish(follow_object.vel);
    ros::spinOnce();
  }
}
