#include <memory>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <memory.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/speed_cmd.hpp"
using std::placeholders::_1;
using namespace std;

#define CCW 0x40
#define RIGHT_WHEEL 0x80
#define BAUD_RATE B19200

class SpeedCommander : public rclcpp::Node
{
    public:
        SpeedCommander() : Node("speed_commander")
        {
            subscription_ = this->create_subscription<custom_msgs::msg::SpeedCmd>(
                "speed_cmds", 10, std::bind(&SpeedCommander::topic_callback, this, _1));

            this->declare_parameter<std::string>("/Port", "/dev/ttyUSB0");
            std::string port;
            this->get_parameter("/Port", port);

            motorController.open(port.c_str(), std::ofstream::binary);
            if(!motorController.is_open())
            {
                std::cout << "Error opening output port. Exiting..." << std::endl;
            }

//            fd = open(port, O_RDWR);

        }

    private:
        SpeedCommander(const SpeedCommander&);
        SpeedCommander& operator=(const SpeedCommander&);

        void topic_callback(custom_msgs::msg::SpeedCmd & msg)
        {
            unsigned char rightCmd = ConvertToHexCmd((-1)*msg.right_speed);
            rightCmd = rightCmd | RIGHT_WHEEL; //to select the right wheel
            //send the command
//            write(fd, rightCmd, 1);
            motorController << rightCmd << flush;

            //build the command
            unsigned char leftCmd = ConvertToHexCmd(msg.left_speed);
            // send the command
            motorController << leftCmd << flush;
        }

        unsigned char ConvertToHexCmd(int speed)
        {
            unsigned char cmd = 0x0;
            // get the magnitude of the speed (if needed) and the direction
            // if the speed is negative we are going CCW (otherwise CW)
            if(speed < 0) {
                int speedMagnitude = -1*speed;
                memcpy(&cmd, &speedMagnitude, sizeof(int)/4);
                cmd = cmd | CCW;
            } else {
                memcpy(&cmd, &speed, sizeof(int)/4);
            }

            return cmd;
        }

        rclcpp::Subscription<custom_msgs::msg::SpeedCmd>::SharedPtr subscription_;
        int fd; // motorcontroller
        ofstream motorController;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedCommander>());
    rclcpp::shutdown();
    return 0;
}