//Joshua Kortje
//Main Wheel Control Implementation
//October 2020

#include "wheelControl.h"
#include <signal.h>
#include <iostream>
#include <cmath>

//Singleton getInstance method
wheelControl* wheelControl::getInstance(int argc, char** argv, const std::string topic)
{
    //Check if there is already an instance
    if(instance == nullptr)
    {
        // Initialize the ros node before we instantiate the class
        ros::init(argc, argv, "wheelController");
    
        instance = new wheelControl(argc, argv, topic);
    }

    return instance;
}

//This class will follow the Singleton pattern
wheelControl::wheelControl(int& argc, char** argv, const std::string topic)
{
    //Initialize the ROS subscriber
    this->subscriber = node.subscribe(topic, 10, &wheelControl::receiveMsg, this);
   
    //The robot starts in a stopped state
    this->currRightSpeed = 0;
    this->currLeftSpeed = 0;

    //Default values
    double lineKp = -0.05;
    double lineKd = 0.0;
    double lineKi = 0.0;
    double objKp = 0.025;
    double objKd = 1000.0;
    double objKi = 0.0;
    double gpsKp = 2.5;
    double gpsKd = 0.0;
    double gpsKi = 0.0;

    //Get the PID terms from system parameters
    if(ros::param::has("/LineCtrlKp")) {
        ros::param::get("/LineCtrlKp", lineKp);
    } else {
        ROS_WARN("Line Ctrl Kp not found. Using default");
    }

    if(ros::param::has("/LineCtrlKd")) {
        ros::param::get("/LineCtrlKd", lineKd);
    } else {
        ROS_WARN("Line Ctrl Kd not found. Using default");
    }

    if(ros::param::has("/LineCtrlKi")) {
        ros::param::get("/LineCtrlKi", lineKi);
    } else {
        ROS_WARN("Line Ctrl Ki not found. Using default");
    }

    if(ros::param::has("/ObjCtrlKp")) {
        ros::param::get("/ObjCtrlKp", objKp);
    } else {
        ROS_WARN("Obj Ctrl Kp not found. Using default");
    }

    if(ros::param::has("/ObjCtrlKd")) {
        ros::param::get("/ObjCtrlKd", objKd);
    } else {
        ROS_WARN("Obj Ctrl Kd not found. Using default");
    }

    if(ros::param::has("/ObjCtrlKi")) {
        ros::param::get("/ObjCtrlKi", objKi);
    } else {
        ROS_WARN("Obj Ctrl Ki not found. Using default");
    }

    if(ros::param::has("/GpsCtrlKp")) {
        ros::param::get("/GpsCtrlKp", gpsKp);
    } else {
        ROS_WARN("GPS Ctrl Kp not found. Using default");
    }

    if(ros::param::has("/GpsCtrlKd")) {
        ros::param::get("/GpsCtrlKd", gpsKd);
    } else {
        ROS_WARN("GPS Ctrl Kd not found. Using default");
    }

    if(ros::param::has("/GpsCtrlKi")) {
        ros::param::get("/GpsCtrlKi", gpsKi);
    } else {
        ROS_WARN("GPS Ctrl Ki not found. Using default");
    }

    //Get the default wheel speed
    if(ros::param::has("/DefaultSpeed")) {
        ros::param::get("/DefaultSpeed", this->defaultSpeed);
    } else {
        ROS_WARN("Default wheel speed not found. Defaulting to 15");
        this->defaultSpeed = 15;
    }

    //Get the speed boost increase
    //This value will be added to the wheel speed during a long
    //straight section of the course if the robot has not turned in a while
    if(ros::param::has("/BoostIncrease")) {
        ros::param::get("/BoostIncrease", this->speedBoost);
    } else {
        ROS_WARN("Wheel boost speed not found. Defaulting to 15");
        this->speedBoost = 15;
    }

    //Set teh number of consecutive times the robot must be keeping to the line
    //to activate the boost
    if(ros::param::has("/BoostCountThreshhold")) {
        ros::param::get("/BoostCountThreshhold", this->boostCountThreshhold);
    } else {
        ROS_WARN("Boost count threshhold not found. Defaulting to 20");
        this->boostCountThreshhold = 20;
    }

    //Initialize the boost count to 0 (no boost at the start)
    this->boostCount = 0;

    this->pidCtrLine = new pidController(lineKp, lineKd, lineKi, this->defaultSpeed, -1.0*this->defaultSpeed, true);
    this->pidCtrObj = new pidController(objKp, objKd, objKi, this->defaultSpeed, -1.0*this->defaultSpeed);
    this->pidCtrGps = new pidController(gpsKp, gpsKd, gpsKi, this->defaultSpeed, -1.0*this->defaultSpeed);
    
    this->wheelDriver = new wheels();
    signal(SIGINT, wheelControl::signalCatch);
    //Start in a None State
    //This means the main controller must come up before 
    //we receive commands
    this->followingMode = FollowMode::eeNone;
    
    //Get the following polarity
    if(ros::param::has("/FollowingDirection")) {
        int param = 0;
        ros::param::get("/FollowingDirection", param);
        this->followingPol = static_cast<FollowPolarity>(param);
    } else {
        ROS_WARN("Following Polarity not found. Defaulting to right");
	    this->followingPol = FollowPolarity::eeRight;
    }

    //Get the distance to follow the line from
    if(ros::param::has("/LineDist")) {
        ros::param::get("/LineDist", this->targetLineDist);
    } else {
        ROS_WARN("Target Line Distance not found. Defaulting to 125");
        this->targetLineDist = 125;
    }

    //Get the distance to follow targets from
    if(ros::param::has("/SideObjectDist")) {
        ros::param::get("/SideObjectDist", this->targetObjDist);
    } else {
        ROS_WARN("Target Side Object Distance not found. Defaulting to 600");
        this->targetObjDist = 600;
    }

    //Margin of error needed in Line Following to increment the boostCount
    if(ros::param::has("/LineBoostMargin")) {
        ros::param::get("/LineBoostMargin", this->lineBoostMargin);
    } else {
        ROS_WARN("Line Boost Margin not found. Defaulting to 30.0");
        this->lineBoostMargin = 30.0;
    }

    //Margin of error needed in GPS Following to increment the boostCount
    if(ros::param::has("/GpsBoostMargin")) {
        ros::param::get("/GpsBoostMargin", this->gpsBoostMargin);
    } else {
        ROS_WARN("GPS Boost Margin not found. Defaulting to 10 degrees");
        this->gpsBoostMargin = 0.1745; //10 degrees in radians
    }

}

wheelControl::~wheelControl() {}

//This function will start the actual control of the wheels
//This is when the wheel Controller acually starts listening to the 
//messages coming in
//NOTE: This is a blocking function
void wheelControl::startControl() {
    ros::spin();
}

//This function will receive a message and send a command down to the wheels
//If the command sent is abouve STOP_CODE_THRESHOLD then a stop command 
//will be send to the wheels to halt the robot.
void wheelControl::receiveMsg(const std_msgs::String::ConstPtr& msg) {
    //Start the left and right speeds at the current values
    int leftSpeed = this->currLeftSpeed;
    int rightSpeed = this->currRightSpeed;
    //Override ramping the speed on a stop command
    bool stopOverride = false;
    //Received a valid message (from the source we are listening to)
    bool messageValid = false;

    ROS_INFO("Received Message: %s", msg->data.c_str());
    //Get the data
    std::string message = msg->data.c_str();
    std::string sender = message.substr(0,3);
    //Check for a stop code or changing state
    if(sender == SWITCH_TO_OBJECT_AVOIDANCE) {
        this->followingMode = FollowMode::eeObject;
        this->boostCount = 0;
        messageValid = true;
        ROS_DEBUG("Switched to Object Avoidance");
    } else if(sender == SWITCH_TO_LINE_FOLLOWING) {
        this->followingMode = FollowMode::eeLine;
        this->boostCount = 0;
        messageValid = true;
        ROS_INFO("Switched to Line Following");
    } else if(sender == SWITCH_TO_GPS_NAV) {
        this->followingMode = FollowMode::eeGps;
        this->boostCount = 0;
        messageValid = true;
        ROS_INFO("Switched to GPS Navigation");
    } else if(sender == SWITCH_TO_TRANSITION) {
        this->followingMode = FollowMode::eeTransition;
        this->boostCount = 0;
        messageValid = true;
        ROS_INFO("Switched to Transition State");
    } else if(sender == TRANSITION_SENDER) {
        //transition state handled seperately
        std::string cmds = message.substr(4);
        std::size_t pos = cmds.find(",");
        messageValid = true;
        ROS_INFO("Transition Received");
        if(pos == std::string::npos) {
            ROS_ERROR("ERROR: Misformatted string");
            return;
        } else {
            leftSpeed = std::stoi(cmds.substr(0, pos));
            rightSpeed = std::stoi(cmds.substr(pos+1));
        }    
    } else if(this->followingMode == FollowMode::eeLine && sender == LINE_SENDER) {
        double position = std::stod(message.substr(4));
        messageValid = true;
        //Use the PID controller to get a command for the wheels
        if(position >= STOP_CODE) {
            leftSpeed = 0;
            rightSpeed = 0;
            stopOverride = true;
            this->boostCount = 0;
        } else {
            messageValid = true;

            //Calculate the error
            double positionError = this->targetLineDist - position;

            //Calculate the differential and apply it to the default speed
            double delta = this->pidCtrLine->control(positionError);
            delta = delta * (double)(this->followingPol);
            leftSpeed = this->defaultSpeed + delta;
            rightSpeed = this->defaultSpeed - delta;

            //Check if we should are in the acceptable zone for picking up speed.i
            if(std::abs(positionError) <= this->lineBoostMargin) {
                this->boostCount++;
            }
            else {
                //if we are not in the acceptable range, we cannot speed up
                this->boostCount = 0;
            }
	        ROS_INFO("Wheel PID Line Controller");
        }
    } else if(this->followingMode == FollowMode::eeObject && sender == OBJ_SENDER) {
        messageValid = true;

        double position = std::stod(message.substr(4));
        //Use the PID controller to get a command for the wheels
        if(position >= STOP_CODE) {
            leftSpeed = 0;
            rightSpeed = 0;
            stopOverride = true;
            this->boostCount = 0;
        }
        else {
            //calculate the differential and apply it to the default speed
            double delta = this->pidCtrObj->control(this->targetObjDist - position); 
            delta = delta * (double)(this->followingPol);
            leftSpeed = this->defaultSpeed + delta;
            rightSpeed = this->defaultSpeed - delta;

            //The boosting functionality is not used in object following
	        ROS_INFO("Wheel PID Obj Controller");
        }
    } else if(this->followingMode == FollowMode::eeGps && sender == GPS_SENDER) {
        messageValid = true;
        double position = std::stod(message.substr(4));
        //Use the PID controller to get a command for the wheels
        if(position >= STOP_CODE) {
            leftSpeed = 0;
            rightSpeed = 0;
            stopOverride = true;
            this->boostCount = 0;
        } else {
            //GPS sends the error already, so calculate the differential
            //and apply it to the wheels
            double delta = this->pidCtrGps->control(position); //GPS sends the error already
            delta = delta * (double)(this->followingPol);
            leftSpeed = this->defaultSpeed + delta;
            rightSpeed = this->defaultSpeed - delta;

            //Check if we can start picking up speed
            if(std::abs(position) <= this->gpsBoostMargin) {
                this->boostCount++;
            }
            else {
                //if we are not in the acceptable range, we cannot speed up
                this->boostCount = 0;
            }
	        ROS_INFO("Wheel PID GPS Controller");
        }
    }

    //TODO debug use only
    ROS_INFO_STREAM("Calculated values: left->" << leftSpeed << " right->" << rightSpeed);
//    ROS_WARN_STREAM("Boost Count: " << this->boostCount);

    //Check if the boost count has met the threshhold to apply the boost
    //The PID controllers should keep giving the same values so the boost
    //can simply add a constant value to both wheels
    if(this->boostCount >= this->boostCountThreshhold && messageValid) {
        leftSpeed += this->speedBoost;
        rightSpeed += this->speedBoost;
    }

    //Send the command to the wheels
    if((leftSpeed != currLeftSpeed) || (rightSpeed != currRightSpeed) && messageValid) {
        //If not a stop Override, make sure we only change by a maximum of 5
        if(!stopOverride)
        {
            //Check that the left speed isn't changing too fast
            if(leftSpeed > this->currLeftSpeed + MAX_CHANGE) {
                leftSpeed = this->currLeftSpeed + MAX_CHANGE;
            } else if(leftSpeed < this->currLeftSpeed - MAX_CHANGE) {
                leftSpeed = this->currLeftSpeed - MAX_CHANGE;
            }

            //Check that the right speed isn't changing too fast
            if(rightSpeed > this->currRightSpeed + MAX_CHANGE) {
                rightSpeed = this->currRightSpeed + MAX_CHANGE;
            } else if(rightSpeed < this->currRightSpeed - MAX_CHANGE) {
                rightSpeed = this->currRightSpeed - MAX_CHANGE;
            }
        }
        ROS_INFO_STREAM("Used values: left->" << leftSpeed << " right->" << rightSpeed);
        this->wheelDriver->CtrlWheels(leftSpeed, rightSpeed);
    }

    //Update the current Speed
    this->currLeftSpeed = leftSpeed;
    this->currRightSpeed = rightSpeed;
}

//This function will perform exit operations if the program is killed in mid-execution
void wheelControl::signalCatch(int signum) {
    ROS_INFO("Shutting down Wheel Controller...");
    //send command to turn off the wheels
    instance->wheelDriver->CtrlWheels(0,0);
    delete instance->pidCtrLine;
    delete instance->pidCtrObj;
    delete instance->pidCtrGps;
    delete instance;
    ros::shutdown();
    exit(signum);
}

//Main for compilation and test
int main(int argc, char** argv) {
    wheelControl* wheelsCtr = wheelsCtr->getInstance(argc, argv, WHEEL_TOPIC);

    ROS_INFO("Wheels now listening...");
    wheelsCtr->startControl();//This call is blocking

    delete wheelsCtr;
    ros::shutdown();
    return 0;
}
