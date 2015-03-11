/*
 * GPIO handler for Dragonboard
 * Author: Kenny Yokoyama & Mike Lara
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"

#define GPIO_EXPORT_PATH    "/sys/class/gpio/export"
#define GPIO_UNEXPORT_PATH  "/sys/class/gpio/unexport"
#define GPIO_DIR_PATH       "/sys/class/gpio/gpio%d/direction"
#define GPIO_VALUE_PATH     "/sys/class/gpio/gpio%d/value"

#define GPIO_HIGH   1
#define GPIO_LOW    0
#define GPIO_IN     "in"
#define GPIO_OUT    "out"
#define PTR_X_MAX   640

enum GPIO_PINS { GPIO_1=6, GPIO_2=7, GPIO_3=206, GPIO_4=207,
    GPIO_5=186, GPIO_6=189, GPIO_7=22, GPIO_8=23};
int PIN_VALUES[5];
enum PIN_DIR { LEFT, LEFT45, FRONT, RIGHT45, RIGHT };

class GPIOPin
{
    private:
        // Mike:
        int hwPinId;
        int fileDesc;

    public:
        GPIOPin(GPIO_PINS pin, const char* pinIO);
        ~GPIOPin();
        // Kenny:
        void setPin(int state);
        // Mike:
        void writeFile(int value);
};

GPIOPin::GPIOPin(GPIO_PINS pin, const char* pinIO)
{
    char path[256];

    hwPinId = pin;
    // setup export
    fileDesc = open(GPIO_EXPORT_PATH, O_WRONLY);
    memset(path, 0, sizeof(path));
    snprintf(path, sizeof(path), "%d", hwPinId);
    write(fileDesc, path, strlen(path));
    close(fileDesc);
    // setup signal direction
    memset(path, 0, sizeof(path));
    snprintf(path, sizeof(path), GPIO_DIR_PATH, hwPinId);
    fileDesc = open(path, O_WRONLY);
    write(fileDesc, pinIO, sizeof(const char*));
    close(fileDesc);
    // open descriptor to value
    memset(path, 0, sizeof(path));
    snprintf(path, sizeof(path), GPIO_VALUE_PATH, hwPinId);
    fileDesc = open(path, O_RDWR);
}

GPIOPin::~GPIOPin()
{
    close(fileDesc);
    //fileDesc = open(GPIO_EXPORT_PATH, O_WRONLY);
    //write(fileDesc, hwPinId, sizeof(hwPinId));
    //close(fileDesc);
}

void GPIOPin::setPin(int state)
{
    char buffer[4];
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer), "%d", state);
    //ROS_INFO("Setting pin %d to %s", hwPinId, buffer);
    lseek(fileDesc, 0, SEEK_SET);
    write(fileDesc, buffer, strlen(buffer));
}

void pinCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    memset(PIN_VALUES, GPIO_LOW, sizeof(PIN_VALUES));
    // Determine which block the point falls into
    if (msg->x < PTR_X_MAX/5) {
	ROS_INFO("LEFT ON");
        PIN_VALUES[LEFT] = GPIO_HIGH;
    } else if (msg->x < 2*PTR_X_MAX/5) {
	ROS_INFO("LEFT 45 ON");
        PIN_VALUES[LEFT45] = GPIO_HIGH;
    } else if (msg->x < 3*PTR_X_MAX/5) {
	ROS_INFO("FRONT ON");
        PIN_VALUES[FRONT] = GPIO_HIGH;
    } else if (msg->x < 4*PTR_X_MAX/5) {
	ROS_INFO("RIGHT 45 ON");
        PIN_VALUES[RIGHT45] = GPIO_HIGH;
    } else {
	ROS_INFO("RIGHT ON");
        PIN_VALUES[RIGHT] = GPIO_HIGH;
    }
}

// Mike:
int main (int argc, char *argv[])
{
    ros::init(argc, argv, "gpio_controller");
    ros::NodeHandle n;
    ros::Subscriber gpio_sub = n.subscribe<geometry_msgs::Point>("/copter_center_2d", 10, &pinCallback);

    ros::Rate loop_rate(10);

    memset(PIN_VALUES, GPIO_LOW, sizeof(PIN_VALUES));

    // create pins
    GPIOPin left(GPIO_2, GPIO_OUT);		// 7
    GPIOPin left_mid(GPIO_1, GPIO_OUT);		// 6
    GPIOPin mid(GPIO_4, GPIO_OUT);		// 207
    GPIOPin right_mid(GPIO_3, GPIO_OUT);	// 206
    GPIOPin right(GPIO_7, GPIO_OUT);		// 22

    while (ros::ok()) {
        left.setPin(PIN_VALUES[LEFT]);
        left_mid.setPin(PIN_VALUES[LEFT45]);
        mid.setPin(PIN_VALUES[FRONT]);
        right_mid.setPin(PIN_VALUES[RIGHT45]);
        right.setPin(PIN_VALUES[RIGHT]);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
