/*
 * GPIO handler for Dragonboard
 * Author: Kenny Yokoyama & Mike Lara
 */
#ifndef __GPIO_H__
#define __GPIO_H__

#define GPIO_EXPORT_PATH  "/sys/class/gpio/export"
#define GPIO_MODE_PATH    "/sys/class/gpio/gpio%d/direction"
#define GPIO_VALUE_PATH   "/sys/class/gpio/gpio%d/value"

#define GPIO_HIGH 1
#define GPIO_LOW  0
#define GPIO_IN   "in"
#define GPIO_OUT  "out"

#define OUTPUT 1
#define INPUT 0

#define GPIO_NUM_COUNT  8
#define GPIO_1  6					// 21
#define GPIO_2  7					// 22
#define GPIO_3  206					// 23
#define GPIO_4  207					// 24
#define GPIO_5  186					// 25
#define GPIO_6  189					// 26
#define GPIO_7  22					// 27
#define GPIO_8  23					// 28

class GPIOPin
{
  private:
  	// Mike:
  	int pinID;
  	int linuxPinPID;
  	int mode;
  	int state;
  	int fileID;
  	//int pinMode[GPIO_NUM_COUNT];
  	//int pinState[GPIO_NUM_COUNT];
    
    // Kenny:
    //int pinDesc[GPIO_NUM_COUNT];
    //int pinData[GPIO_NUM_COUNT];

  public:
  	// Kenny:
    void initPin();
    void setPin(int state);
    // Mike:
    int getPinState();
    GPIOPin(int pinNumber);
    ~GPIOPin();
    void setPinMode(int pinMode);
    int getPinMode();
    void writeFile(int value);
};

#endif