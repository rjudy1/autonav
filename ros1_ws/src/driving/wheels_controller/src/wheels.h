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

using namespace std;

#define PORT "/dev/ttyUSB0"
#define CCW 0x40
#define RIGHT_WHEEL 0x80
#define BAUD_RATE B19200

class wheels
{
    public:
    wheels();
    
    ~wheels();
    
    //This function takes a left and right speed and
    //controls each wheel independantly
    //The speeds are positive for forward and 
    //negative for backwards direction. The speeds are defined as
    //being a number with a magnitude less than or equal to 63.
    void CtrlWheels(int leftSpeed, int rightSpeed);

    private:
    //Control the right wheel
    void CtrlRightWheel(int rightSpeed);

    //Control the left wheel
    void CtrlLeftWheel(int leftSpeed);
    
    //This function will take in an integer speed and convert it to 
    //a hexidecimal command that can be interpretted by the 
    //motor controller
    //Note: this function does not take care of the 
    //channel parameter
    unsigned char ConvertToHexCmd(int speed);

    //Hidden copy constructor and copy assignment operator
    wheels(const wheels&);
    wheels& operator=(const wheels&);
    
    //Variables
    ofstream motorController;
};



