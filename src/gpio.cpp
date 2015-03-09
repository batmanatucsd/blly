/*
 * GPIO handler for Dragonboard
 * Author: Kenny Yokoyama & Mike Lara
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ros/ros.h"

#define GPIO_EXPORT_PATH    "/sys/class/gpio/export"
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
    fileDesc = open(GPIO_EXPORT_PATH, O_RDWR);
    memset(path, 0, sizeof(path));
    snprintf(path, sizeof(path), "%d", hwPinId);
    write(fileDesc, path, strlen(path));
    close(fileDesc);
    // setup signal direction
    memset(path, 0, sizeof(path));
    snprintf(path, sizeof(path), GPIO_DIR_PATH, hwPinId);
    fileDesc = open(path, O_RDWR);
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
}

void GPIOPin::setPin(int state)
{
    char buffer[4];
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer), "%c", state);
    lseek(fileDesc, 0, SEEK_SET);
    write(fileDesc, buffer, sizeof(buffer));
}

void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
    memset(PIN_VALUES, GPIO_LOW, sizeof(PIN_VALUES));
    if (!(msg < 0 || msg > sizeof(PIN_VALUES)/sizeof(int))) {
        PIN_VALUES[msg] = GPIO_HIGH;
        ROS_INFO("%d is now on", msg);
    }
}

// Mike:
int main (int argc, char *argv[])
{
    ros::init(argc, argv, "gpio_controller");
    ros::NodeHandle n;
    ros::Subscriber gpio_sub = n.subscribe<std_msgs::Int32>("/gpio_ctl", 10, &handlePin);

    ros::Rate loop_rate(1);

    memset(PIN_VALUES, GPIO_LOW, sizeof(PIN_VALUES));

    // create pins
    GPIOPin pin1(GPIO_1, GPIO_OUT);
    GPIOPin pin2(GPIO_2, GPIO_OUT);
    GPIOPin pin3(GPIO_3, GPIO_OUT);
    GPIOPin pin4(GPIO_4, GPIO_OUT);
    GPIOPin pin7(GPIO_7, GPIO_OUT);

    while (ros::ok()) {
        pin1.setPin(PIN_VALUES[0]);
        pin2.setPin(PIN_VALUES[1]);
        pin3.setPin(PIN_VALUES[2]);
        pin4.setPin(PIN_VALUES[4]);
        pin7.setPin(PIN_VALUES[5]);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
