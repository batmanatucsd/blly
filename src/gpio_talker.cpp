#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"

#include <sstream>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gpio_talker");
    ros::NodeHandle n;

    ros::Publisher talker_pub = n.advertise<geometry_msgs::Point>("/gpio_ctl", 10);
    ros::Rate loop_rate(1);

    int count = -10;
    while (ros::ok()) {
        geometry_msgs::Point pnt;
        pnt.x = count;
        pnt.z = 10;
        talker_pub.publish(pnt);

        ROS_INFO("%d", count);

        count = (count > 10) ? count + 1: -10;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
