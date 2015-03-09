#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <sstream>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gpio_talker");
    ros::NodeHandle n;

    ros::Publisher talker_pub = n.advertise<std_msgs::Int32>("/gpio_ctl", 10);
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok()) {
	std_msgs::Int32 msg;
	msg.data = count;
        talker_pub.publish(msg);

        ROS_INFO("%d", count);

        ros::spinOnce();
        loop_rate.sleep();
        count = (count + 1) % 5;
    }

    return 0;
}
