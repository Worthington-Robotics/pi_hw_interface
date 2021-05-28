#include <tinyxml.h>

#include <chrono>
#include <exception>
#include <filesystem>
#include <gpiod.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#define SAFETY_TIMEOUT 100ms
#define SAFETY_TIMEOUT_CHECK 10ms

using std::placeholders::_1;
using namespace std::chrono_literals;

struct GpioPort {
    int port;
    std::string chip = "gpiochip0";
    short pinDir = 0; //0 is output, 1 is input, 2 is safety and any other is as is
    short pullDir = 0;  // 0 is no change, 1 is up 2 is down
};

class HardwareController : public rclcpp::Node {
private:
    //map of all gpio lines requested by system
    std::map<std::shared_ptr<GpioPort>, std::shared_ptr<gpiod::line>> lines;

    //list of all subscribed topics
    std::vector<std::string> topics;

    //subscriptions for all motor inputs
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> lineSubscriptions;

    //safety enable subscription that allows motors to be active
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;

    //safety enable timer
    rclcpp::TimerBase::SharedPtr safetyTimer;

    //flag for saftey enable of all outputs
    bool safetyEnable = false;

    //last time the safety stamp was set to true
    std::chrono::_V2::system_clock::duration safetyStamp;

public:
    HardwareController() : Node("pi_hw_interface") {
        safetySubscrip = create_subscription<std_msgs::msg::Bool>("safety_enable", 10, std::bind(&HardwareController::feedSafety, this, _1));
        safetyStamp = std::chrono::system_clock::now().time_since_epoch();
        safetyTimer = create_wall_timer(SAFETY_TIMEOUT_CHECK, std::bind(&HardwareController::safetyTimerUpdate, this));
    }

    void registerGpio(std::vector<std::shared_ptr<GpioPort>> ports) {
        try {
            //create all GPIO
            for (auto it = ports.begin(); it != ports.end(); it++) {
                std::shared_ptr<gpiod::chip> gpioChip = std::make_shared<gpiod::chip>(it->get()->chip);
                std::shared_ptr<gpiod::line> line = std::make_shared<gpiod::line>(it->get()->port);
                int dir = 0;
                switch (it->get()->pinDir) {
                case 0:
                    dir = gpiod::line_request::DIRECTION_INPUT;
                    break;
                case 1:
                    dir = gpiod::line_request::DIRECTION_OUTPUT;
                    break;
                case 2:
                    dir = gpiod::line_request::DIRECTION_OUTPUT;
                    break;
                default:
                    dir = gpiod::line_request::DIRECTION_AS_IS;
                }

                std::bitset<32UL> flags = 0;
                switch (it->get()->pullDir)
                {
                case 1:
                    flags |= gpiod::line_request::FLAG_BIAS_PULL_UP;
                    break;
                case 2:
                    flags |= gpiod::line_request::FLAG_BIAS_PULL_DOWN;
                    break;
                default:
                    flags |= gpiod::line_request::FLAG_BIAS_DISABLE;
                    break;
                }

                line->request({"hw_interface", dir, flags});

                lines->[it] = line;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind Gpio\nCause: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind Gpio\nCause Unknown");
        }
    }

    void feedSafety(std::shared_ptr<std_msgs::msg::Bool> msg) {
        if (msg->data) {
            safetyStamp = std::chrono::system_clock::now().time_since_epoch();
            safetyEnable = true;
        }
    }

    void safetyTimerUpdate() {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        if (safetyEnable && (now - SAFETY_TIMEOUT > safetyStamp))
            safetyEnable = false;
    }
};

int main(int argc, char** argv) {
    //init ros node
    rclcpp::init(argc, argv);
    std::shared_ptr<HardwareController> rosNode = std::make_shared<HardwareController>();
    RCLCPP_INFO(rosNode->get_logger(), "hardware interface node starting");

    // Load the xml file
    std::filesystem::path config = std::filesystem::current_path() / "config.xml";
    TiXmlDocument* doc = new TiXmlDocument(config.c_str());
    if (!doc->LoadFile()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Error parsing XML config %s\n %s", config.c_str(), doc->ErrorDesc());

    } else {
        try {
            //std::shared_ptr<std::vector<robotmotors::MotorMap>> motors = robotmotors::createMotorMap(doc);
            //RCLCPP_INFO(rosNode->get_logger(), "Recieved config for %d motor(s)", motors->size());
            //rosNode->setMotors(motors);

            //set all motors to neutral
            //rosNode->neutralMotors();

            RCLCPP_INFO(rosNode->get_logger(), "hardware interface node loaded using gpiod interface");

            // serve the callbacks
            rclcpp::spin(rosNode);

            RCLCPP_INFO(rosNode->get_logger(), "hardware interface shutting down");

            //set all motors to neutral
            //rosNode->neutralMotors();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rosNode->get_logger(), "Node failed\nCause: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(rosNode->get_logger(), "Node failed\nCause Unknown");
        }
    }

    delete doc;

    RCLCPP_INFO(rosNode->get_logger(), "hardware interface shut down complete");

    rclcpp::shutdown();

    return 0;
}