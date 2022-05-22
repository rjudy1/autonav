//Joshua Kortje
//PID Controller Class
//September 2020

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define HISTORY_SIZE 10

#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>

class pidController
{
    public:
    //Constructor
    //newKp -proportional gain
    //newKd -derivative gain
    //newKi -integral gain
    //newMax and newMin -max and min values for the controllered variable
    pidController(double newKp, double newKd, double newKi, double newMax, double newMin, bool record=false)
    {
        Kp = newKp;
        Kd = newKd;
        Ki = newKi;
        maxValue = newMax;
        minValue = newMin;
        prevError = 0.0;
        std::fill(std::begin(prevIntegralErrors), std::end(prevIntegralErrors), 0.0);
        recording = record;
        if(recording)
        {
            outputFile.open("/home/autonav/autonav_ws/pidvalues.csv");
            outputFile << "Curr,Derivative,Integral,Ouput,Iteration Time\n";
            outputFile << std::to_string(Kp) << "," << std::to_string(Kd) << ","
                << std::to_string(Ki) << "\n";
        }
        //initialize the first time to when the PID controller starts up.
        //this should give a sufficiently large value to not take the 
        //first value into account at first
        lastTime = std::chrono::steady_clock::now();
    }

    //Default constructor
    pidController(): pidController(0.1, 0.0, 0.0, 63.0, -63.0, false)
    {
    }


    //Destructor
    ~pidController() 
    {
        if(recording)
        {
            outputFile.close();
        }
    }

    //Control function
    //This function takes in the desired value and the current value
    //and after doing some calculations, returns the
    //manipulated value.
    double control(double currError);
    
    private:
    //variable for if we want to record a file with the PID values for analysis
    bool recording;
    std::ofstream outputFile;
    //time of last iteration for the controller
    std::chrono::steady_clock::time_point lastTime;
    //PID Parameters
    double Kp;
    double Kd;
    double Ki;
    
    //Max and Min values for the variable
    double maxValue;
    double minValue;

    //history variables for memory
    double prevError;
    double prevIntegralErrors[HISTORY_SIZE];
};

#endif //PID_CONTROLLER_H
