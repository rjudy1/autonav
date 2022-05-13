//Joshua Kortje
//Wheel Controller Implimentation
//September 2020

#include <iostream>
#include "pidController.h"

//Control function
double pidController::control(double currError)
{
    //figure out the time since the last iteration
    std::chrono::steady_clock::time_point currTime = std::chrono::steady_clock::now();    
    double iterationTime = (std::chrono::duration_cast<std::chrono::microseconds>(currTime - lastTime)).count();
    std::cout << "Iteration Time in microseconds: " << iterationTime << std::endl;

    //calculate the I term
    double prevIntError = 0.0;
    for(int i = 0; i < HISTORY_SIZE; i++)
    {
        prevIntError = prevIntError + prevIntegralErrors[i];
    }
    double integralTerm = prevIntError+currError*iterationTime;
    
    //calculate the D term
    double derivativeTerm = (currError - prevError)/iterationTime;

    //calculate the output
    double output = Kp*currError + Ki*integralTerm + Kd*derivativeTerm;

    //Update all of the previous values
    prevError = currError;
    //update the integral history buffer
    //shift everything over and add the new one
    for(int i = 0; i < (HISTORY_SIZE - 1); i++)
    {
        prevIntegralErrors[i] = prevIntegralErrors[i+1];
    }
    prevIntegralErrors[HISTORY_SIZE - 1] = currError*iterationTime;

    //check bounds
    if(output > maxValue)
    {
        output = maxValue;
    }
    else if(output < minValue)
    {
        output = minValue;
    }

    //save the last time for the next one
    lastTime = currTime;

    if(recording)
    {
        outputFile << std::to_string(currError) << "," << std::to_string(derivativeTerm) << "," 
            << std::to_string(integralTerm) << "," << std::to_string(output) 
            << "," << std::to_string(iterationTime) << "\n";
    }

    return output;
}

/*
//Fake Main for compilation and test
int main()
{
    std::cout << "Enter number greater than 100 to stop. Otherwise, send command to wheels." << std::endl;
    pidController wheelsCtl;
    wheels* wheelDriver = new wheels();
    double error = 0.0;
    double constant = 10.0; //set speed for the motors (10 = 1 mph?)
    double delta = 0.0;
    std::cin >> error;
    while (error < 100)
    {
        //threshold the input given
	if(error > 63.0)
	{
		error = 63.0;
	}
	else if(error < -63.0)
	{
		error = -63.0;
	}

	//send the command to the wheels
	delta = wheelsCtl.control(error);
        wheelDriver->CtrlWheels(constant - delta, constant + delta);
	std::cout << "Target: " << error << std::endl;
	std::cout << "Current Speed: " << delta << std::endl << std::endl;

	//get the new value
	std::cout << "Enter a new value." << std::endl;
	std::cin >> error;
    }
   
    wheelDriver->CtrlWheels(0,0); 
    delete wheelDriver;
    return 0;
}
*/
