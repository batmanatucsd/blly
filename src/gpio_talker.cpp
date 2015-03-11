#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"

#include <sstream>

#define PTR_X_MAX   650

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gpio_talker");
    ros::NodeHandle n;

    ros::Publisher talker_pub = n.advertise<geometry_msgs::Point>("/copter_center_2d", 10);
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok()) {
        geometry_msgs::Point pnt;
        pnt.x = count;
        pnt.z = 10;
        talker_pub.publish(pnt);

        ROS_INFO("%d", count);

        count = (count + 10) % PTR_X_MAX;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
