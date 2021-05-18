#include <iostream>
#include <gpiod.hpp>
#include <string>


int main(int argc, char** argv){

    std::cout << "Hello" << std::endl;
    gpiod::chip chip0("gpiochip0");

    std::cout << "line 4: " << chip0.get_line(4).get_value() << std::endl;

}