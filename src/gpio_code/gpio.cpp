/*
 * GPIO handler for Dragonboard
 * Author: Kenny Yokoyama & Mike Lara
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "gpio.h"

#define GPIO_EXPORT_PATH    "/sys/class/gpio/export"
#define GPIO_DIR_PATH       "/sys/class/gpio/gpio%d/direction"
#define GPIO_VALUE_PATH     "/sys/class/gpio/gpio%d/value"

#define GPIO_HIGH   1
#define GPIO_LOW    0
#define GPIO_IN     "in"
#define GPIO_OUT    "out"

#define GPIO_1  6
#define GPIO_2  7
#define GPIO_3  206
#define GPIO_4  207
#define GPIO_5  186
#define GPIO_6  189
#define GPIO_7  22
#define GPIO_8  23

class GPIOPin
{
    private:
        // Mike:
        int pinId;
        int hwPinId;
        int fileDesc;

    public:
        GPIOPin(int pinNumber);
        ~GPIOPin();
        // Kenny:
        void setPin(int state);
        // Mike:
        void writeFile(int value);
};

GPIOPin::GPIOPin(int pinNumber, const char* pinIO)
{
    char path[256];

    pinId = pinNumber;
    switch(pinId)
    {
        case 1: hwPinId = GPIO_1; break;
        case 2: hwPinId = GPIO_2; break;
        case 3: hwPinId = GPIO_3; break;
        case 4: hwPinId = GPIO_4; break;
        case 5: hwPinId = GPIO_5; break;
        case 6: hwPinId = GPIO_6; break;
        case 7: hwPinId = GPIO_7; break;
        case 8: hwPinId = GPIO_8; break;
        default: // error case
    }
    // fill path array with 0's
    memset(path, 0, sizeof(path));
    // print GPIO_MODE_PATH's string to path buffer
    snprintf(path, sizeof(path), GPIO_DIR_PATH, hwPinId);
    fileDesc = open(path, O_RDWR);
    // TODO setup export
    // TODO setup signal direction
}

GPIOPin::~GPIOPin()
{
    close(fileDesc);
}

void GPIOPin::setPin(int state)
{
    char buffer[4];
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer), "%c", state);							// ideally, use snprintf()
    lseek(P2I(pinID), 0, SEEK_SET);
    write(P2I(pinID), buffer, sizeof(buffer));
}

// Mike:
int main (int argc, char *argv[])
{
    // create pins
    GPIOPin pin1(GPIO_1, GPIO_OUT);
    GPIOPin pin2(GPIO_2, GPIO_OUT);
    GPIOPin pin3(GPIO_3, GPIO_OUT);
    GPIOPin pin4(GPIO_4, GPIO_OUT);
    GPIOPin pin7(GPIO_7, GPIO_OUT);

    // set all pins low
    pin1.setPin(GPIO_LOW);
    pin2.setPin(GPIO_LOW);
    pin3.setPin(GPIO_LOW);
    pin4.setPin(GPIO_LOW);
    pin7.setPin(GPIO_LOW);
}
