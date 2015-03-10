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

#define GPIO_EXPORT_PATH    "/sys/class/gpio/export"
#define GPIO_UNEXPORT_PATH  "/sys/class/gpio/unexport"
#define GPIO_DIR_PATH       "/sys/class/gpio/gpio%d/direction"
#define GPIO_VALUE_PATH     "/sys/class/gpio/gpio%d/value"

#define GPIO_HIGH   1
#define GPIO_LOW    0
#define GPIO_IN     "in"
#define GPIO_OUT    "out"

enum GPIO_PINS { GPIO_1=6, GPIO_2=7, GPIO_3=206, GPIO_4=207,
    GPIO_5=186, GPIO_6=189, GPIO_7=22, GPIO_8=23};
int PIN_VALUES[5];

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
    ROS_INFO("Setting pin %d to %s", hwPinId, buffer);
    lseek(fileDesc, 0, SEEK_SET);
    write(fileDesc, buffer, strlen(buffer));
}

void pinCallback(const std_msgs::Int32::ConstPtr& msg)
{
    memset(PIN_VALUES, GPIO_LOW, sizeof(PIN_VALUES));
    if (!(msg->data < 0 || msg->data > sizeof(PIN_VALUES)/sizeof(int))) {
        PIN_VALUES[msg->data] = GPIO_HIGH;
        ROS_INFO("%d is now on", msg->data);
    }
}

// Mike:
int main (int argc, char *argv[])
{
    ros::init(argc, argv, "gpio_controller");
    ros::NodeHandle n;
    ros::Subscriber gpio_sub = n.subscribe<std_msgs::Int32>("/gpio_ctl", 10, &pinCallback);

    ros::Rate loop_rate(1);

    memset(PIN_VALUES, GPIO_LOW, sizeof(PIN_VALUES));

    // create pins
    GPIOPin left_mid(GPIO_1, GPIO_OUT);
    GPIOPin left(GPIO_2, GPIO_OUT);
    GPIOPin right_mid(GPIO_3, GPIO_OUT);
    GPIOPin mid(GPIO_4, GPIO_OUT);
    GPIOPin right(GPIO_7, GPIO_OUT);

    while (ros::ok()) {
        left.setPin(PIN_VALUES[0]);
        left_mid.setPin(PIN_VALUES[1]);
        mid.setPin(PIN_VALUES[2]);
        right_mid.setPin(PIN_VALUES[3]);
        right.setPin(PIN_VALUES[4]);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
