#include <iostream>
#include <gpiod.hpp>
#include <string>


int main(int argc, char** argv){

    std::cout << "Hello" << std::endl;
    gpiod::chip chip0("gpiochip0");

    auto line = chip0.get_line(4);

    gpiod::line_request dirConfig = gpiod::line_request();
    dirConfig.consumer = "whee";
    dirConfig.request_type = gpiod::line_request::DIRECTION_INPUT;
    //dirConfig.flags = gpiod::line_request::FLAG_ACTIVE_LOW;   // for active low
    dirConfig.flags = 0;                                        // for active high

    line.request(dirConfig);
    
    if(line.direction() == 1){
        std::cout << "line 4: " << line.get_value() << std::endl;
    } else {
        std::cout << "error setting direction on line 4: " << line.is_requested() << std::endl;
    }

    

}


