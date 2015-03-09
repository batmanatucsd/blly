/*
 * GPIO handler for Dragonboard
 * Author: Kenny Yokoyama & Mike Lara
 */

 /**************************************************
 * Questions for Kenny: 
 * 1) What is P2I?
 * 2) Can you explain memset, snprintf, initPin()?
 # 2b) Can you explain writing to sys file?
 * 3) Why use snprint vs sprint?
 * 4) Is path available to other functions?
 * 5) Best way to implement a ctor?
 ***************************************************/
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "gpio.h"

// Kenny:
void GPIOPin::initPin()
{
  char path[256];										

  switch(pinID)
  {
  	case 1:
  		linuxPinID = GPIO_1;
  		break;
  	case 2:
  		linuxPinID = GPIO_2;
  		break;
  	case 3:
  		linuxPinID = GPIO_3;
  		break;
  	case 4:
  		linxuPinID = GPIO_4;
  		break;
  	case 5:
  		linuxPinID = GPIO_5;
  		break;
  	case 6:
  		linuxPinID = GPIO_6;
  		break;
  	case 7:
  		linuxPinID = GPIO_7;
  		break;
  	case 8:
  		linuxPinID = GPIO_8;
  		break;
  	default:
  		// error case
  }

  memset(path, 0, sizeof(path));						// fill path array with 0's
  snprintf(path, sizeof(path), GPIO_MODE_PATH, linxuPinID);	// print GPIO_MODE_PATH's string to path buffer
  fileID = open(path, O_RDWR);

  // needs to be completed
}

void GPIOPin::setPin(int state)
{
  char buffer[4];
  memset(buffer, 0, sizeof(buffer));
  sprintf(buffer, "%c", state);							// ideally, use snprintf()
  lseek(P2I(pinID), 0, SEEK_SET);
  write(P2I(pinID), buffer, sizeof(buffer));
}

// Mike:
int GPIOPin::getPinState()
{
	return state;
}

GPIOPin::GPIOPin(int pinNumber)
{
	pinID = pinNumber;
}

GPIOPin::~GPIOPin()
{
	//delete[] ;
}

void GPIOPin::setPinMode(int pinMode)
{
	mode = pinMode;
	writeFile(mode);
}

int GPIOPin::getPinMode()
{
	return mode;
}

void GPIOPin::writeFile(int value)
{
	char buffer[4];
	memset((void *)buffer, 0, sizeof(buffer));
	sprintf(buffer, "%c", value);
	lseek(P2I(pinID), 0, SEEK_SET);
	write(P2I(pinID), buffer, sizeof(buffer));
}

int main (int argc, char *argv[])
{
	// create pins
	GPIOPin pin1(GPIO_1);
	GPIOPin pin2(GPIO_2);
	GPIOPin pin3(GPIO_3);
	GPIOPin pin4(GPIO_4);
	GPIOPin pin7(GPIO_7);

	// init pins
	pin1.initPin();
	pin2.initPin();
	pin3.initPin();
	pin4.initPin();
	pin7.initPin();

	// set pins as outputs
	pin1.setPinMode(OUTPUT);
	pin2.setPinMode(OUTPUT);
	pin3.setPinMode(OUTPUT);
	pin4.setPinMode(OUTPUT);
	pin7.setPinMode(OUTPUT);

	// set all pins low
	pin1.setPin(GPIO_LOW);
	pin2.setPin(GPIO_LOW);
	pin3.setPin(GPIO_LOW);
	pin4.setPin(GPIO_LOW);
	pin7.setPin(GPIO_LOW);



}