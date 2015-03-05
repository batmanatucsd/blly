/*
 * GPIO handler for Dragonboard
 * Author: Kenny Yokoyama
 */
#ifndef __GPIO_H__
#define __GPIO_H__

#define GPIO_EXPORT_PATH  "/sys/class/gpio/export"
#define GPIO_DIR_PATH     "/sys/class/gpio/gpio%d/direction"
#define GPIO_VALUE_PATH   "/sys/class/gpio/gpio%d/value"

#define GPIO_HIGH 1
#define GPIO_LOW  0
#define GPIO_IN   "in"
#define GPIO_OUT  "out"

#define GPIO_NUM_COUNT  8
#define GPIO_1  21
#define GPIO_2  22
#define GPIO_3  23
#define GPIO_4  24
#define GPIO_5  25
#define GPIO_6  26
#define GPIO_7  27
#define GPIO_8  28

#define P2I(x) (x-GPIO_1)

class GPIO
{
  private:
    int pinDesc[GPIO_NUM_COUNT];
    int pinData[GPIO_NUM_COUNT];

  public:
    void initPin(int pinID);
    void setPin(int pinID, int state);
};

#endif
