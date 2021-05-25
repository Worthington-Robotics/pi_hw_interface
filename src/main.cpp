#include <iostream>
#include <gpiod.hpp>
#include <string>

extern "C" {
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}



int main(int argc, char** argv){

    const int thresh = 2000;
    int count = 20;

    std::cout << "Hello" << std::endl;
    gpiod::chip chip0("gpiochip0");

    auto line = chip0.get_line(5);

    std::cout << "System got line " << line.offset() << std::endl;

    gpiod::line_request dirConfig = gpiod::line_request();
    dirConfig.consumer = "whee";
    dirConfig.request_type = gpiod::line_request::DIRECTION_INPUT;
    //dirConfig.flags = gpiod::line_request::FLAG_ACTIVE_LOW;        // for active low
    //dirConfig.flags = 0;                                           // for active high
    //dirConfig.flags = gpiod::line_request::FLAG_BIAS_PULL_UP;      // for pull up resistors
    dirConfig.flags = gpiod::line_request::FLAG_BIAS_PULL_DOWN;    // for pull down resistors

    line.request(dirConfig);
    
    if(line.direction() == 1){
        std::cout << "line " << line.offset() << ": " << line.get_value() << std::endl;
    } else {
        std::cout << "error setting direction on line " << line.offset() << ": " << line.is_requested() << std::endl;
    }

    auto out = chip0.get_line(4);
    std::cout << "System got line " << out.offset() << std::endl;
    gpiod::line_request outConfig = gpiod::line_request();
    outConfig.consumer = "wheeOut";
    outConfig.request_type = gpiod::line_request::DIRECTION_OUTPUT;

    out.request(outConfig);

    int i = 0;
    while(count != 0){
        if(i < (thresh / 2)){
            out.set_value(1);
        } else {
            out.set_value(0);
        }


        if(i % thresh == 0){
            i = 0;
            count--;
        } else {
            i++;
        }
    }

    const int address = 0x27;
}


