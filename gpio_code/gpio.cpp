/*
 * GPIO handler for Dragonboard
 * Author: Kenny Yokoyama
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "gpio.h"

void GPIO::initPin(int pinID)
{
  char path[256];
  memset(path, 0, sizeof(path));
  snprintf(path, sizeof(path), GPIO_DIR_PATH, pinID);
}

void GPIO::setPin(int pinID, int state)
{
  char buffer[4];
  memset(buffer, 0, sizeof(buffer));
  sprintf(buffer, "%c", state);
  lseek(P2I(pinID), 0, SEEK_SET);
  write(P2I(pinID), buffer, sizeof(buffer));
}
