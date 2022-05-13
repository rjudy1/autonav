//Joshua Kortje
//Main Wheel Control Class
//October 2020

#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#define SWITCH_TO_TRANSITION "STR"
#define SWITCH_TO_OBJECT_AVOIDANCE "SOA"
#define SWITCH_TO_LINE_FOLLOWING "SLF"
#define SWITCH_TO_GPS_NAV "SGN"
#define STOP_CODE 7777
#define WHEEL_TOPIC "wheel_distance"
#define TRANSITION_SENDER "TRA"
#define LINE_SENDER "LIN"
#define OBJ_SENDER "OBJ"
#define GPS_SENDER "GPS"
#define LINE_FOLLOWING "LINE_FOLLOWING_STATE"
#define OBJECT_AVOIDANCE_FROM_LINE "OBJECT_AVOIDANCE_FROM_LINE_STATE"
#define OBJECT_AVOIDANCE_FROM_GPS "OBJECT_AVOIDANCE_FROM_GPS_STATE"
#define GPS_NAVIGATION "GPS_NAVIGATION_STATE"
#define LINE_TO_OBJECT "LINE_TO_OBJECT_STATE"
#define MAX_CHANGE 5

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "pidController.h"
#include "wheels.cpp"
#include <ros/console.h>
#include <unistd.h>

//This is an enum indicating whether we are left or right
//following
enum FollowPolarity { eeRight = 1, eeLeft = -1};

//This is an enum indicating which mode we are in
enum FollowMode { eeNone, eeLine, eeObject, eeGps , eeTransition};

//This class is a class to control the wheels. It is styled after
//the Singleton pattern and includes a subscriber to a ROS topic
//for receiving messages pertaining to the wheels.
class wheelControl
{
    public:
    //Singleton pattern. This function will call the constructor and 
    //return the single instance of the class
    static wheelControl* getInstance(int argc = 0, char** argv = nullptr, const std::string = "");

    //Destructor
    ~wheelControl();

    //This function will begin control of the wheels
    void startControl();

    //This function will receive a message and send a command to the wheels
    void receiveMsg(const std_msgs::String::ConstPtr& msg);

    //This function is a signal handler that will catch a signal (such as ^C) and turn off the wheels before exiting the program
    static void signalCatch(int signum);

    private:
    //Constructor
    //the constructor needs the starting arguements to initialize ros
    //communication
    wheelControl(int& argc, char** argv, const std::string);

    //Single instance of the class
    static wheelControl* instance;

    //Default wheel speed
    int defaultSpeed;

    //Current Speed of the right wheel
    int currRightSpeed;

    //Current Speed of the left wheel
    int currLeftSpeed;

    //Acceptable margin of error for applying a boost to line following
    double lineBoostMargin;

    //Acceptable margin of error for applying a boost to GPS following
    double gpsBoostMargin;

    //Number of times the error margin must be met before applying a boost
    int boostCountThreshhold;

    //The count of errors in the acceptable range
    //When this value goes above boostCountThreshhold the boost will be triggered
    int boostCount;

    //The speed boost increment added during a boost
    int speedBoost;

    //PID Controller for controlling the wheel speed by line following
    pidController* pidCtrLine;

    //PID Controller for controlling the wheel speed by object avoidance
    pidController* pidCtrObj;

    //PID Controller for controlling the wheel speed by GPS navigation
    pidController* pidCtrGps;

    //This is the wheel driver
    wheels* wheelDriver;

    //ROS Node Handle
    ros::NodeHandle node;

    //Subscriber
    ros::Subscriber subscriber;

    //This value will give us the target distance from the line we are trying to attain
    double targetLineDist;

    //This value will give us the target distance from the object we are tring to keep
    double targetObjDist;

    //Enum indicating if we are left or right following
    FollowPolarity followingPol;

    //Enum indicating what we are following
    FollowMode followingMode;

    //Hidden copy constructor and assignment operator
    wheelControl(const wheelControl&);
    wheelControl& operator=(const wheelControl&);
};

//Define the static Singleton instance pointer
wheelControl* wheelControl::instance = nullptr;

#endif //WHEEL_CONTROL_H
