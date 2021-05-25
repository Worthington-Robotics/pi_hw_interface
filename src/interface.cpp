#include <iostream>
#include <gpiod.hpp>
#include <string>
#include <tinyxml.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


struct Gpio {
    int port;
    std::string chip = "gpiochip0";
};

class HardwareController : public rclcpp::Node {
    private:
    std::map<int
};

int main(int argc, char** argv){

    const int thresh = 2000;
    int count = 20;

    std::cout << "Hello" << std::endl;
}