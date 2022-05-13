//Joshua Kortje
//Sept. 2 2020


//Description: This file defins a set of functions that can be used to control the wheels of the robot. 


#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <memory.h>
#include "wheels.h"

#define PORT "/dev/ttyUSB0"
#define CCW 0x40
#define RIGHT_WHEEL 0x80
#define BAUD_RATE B19200

wheels::wheels()
{
    motorController.open(PORT, std::ofstream::binary);
    if(!motorController.is_open())
    {
        std::cout << "Error opening output port. Exiting..." << std::endl;
    }
}
    
wheels::~wheels()
{
    motorController.close();
}
    
//This function takes a left and right speed and
//controls each wheel independantly
//The speeds are positive for forward and 
//negative for backwards direction. The speeds are defined as
//being a number with a magnitude less than or equal to 63.
void wheels::CtrlWheels(int leftSpeed, int rightSpeed)
{
    CtrlRightWheel(rightSpeed);
    CtrlLeftWheel(leftSpeed);
}

//Control the right wheel
void wheels::CtrlRightWheel(int rightSpeed)
{
    //build the command
    //the polarity of the right wheel must be reversed
    unsigned char rightCmd = ConvertToHexCmd((-1)*rightSpeed);
    rightCmd = rightCmd | RIGHT_WHEEL; //to select the right wheel
    
    //send the command
    motorController << rightCmd << flush;
}

//Control the left wheel
void wheels::CtrlLeftWheel(int leftSpeed)
{
    //build the command
    unsigned char leftCmd = ConvertToHexCmd(leftSpeed);
       
    //send the command
    motorController << leftCmd << flush;
}
    
//This function will take in an integer speed and convert it to 
//a hexidecimal command that can be interpretted by the 
//motor controller
//Note: this function does not take care of the 
//channel parameter
unsigned char wheels::ConvertToHexCmd(int speed)
{
    unsigned char cmd = 0x0;
        
    //get the magnitude of the speed (if needed)
    //and the direction
    //if the speed is negative
    //we are going CCW (otherwise CW)
    if(speed < 0)
    {
        int speedMagnitude = -1*speed;
        memcpy(&cmd, &speedMagnitude, sizeof(int)/4);
        cmd = cmd | CCW;
    }
    else
    {
        memcpy(&cmd, &speed, sizeof(int)/4);
    }
    
    return cmd;
}



