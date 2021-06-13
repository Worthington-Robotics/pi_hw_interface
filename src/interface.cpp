#include <tinyxml.h>

#include <chrono>
#include <exception>
#include <filesystem>
#include <gpiod.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#define SAFETY_TIMEOUT 100ms
#define SAFETY_TIMEOUT_CHECK 10ms

//#define ALLOW_PULL_DIR

using std::placeholders::_1;
using namespace std::chrono_literals;

bool getValue(TiXmlElement* elem, const std::string& childName, std::string& value);
bool stob(std::string s);

struct GpioPort {
    int port;
    std::string chip = "gpiochip0";
    short pinDir = 0;   //0 is output, 1 is input, 2 is safety and any other is as is
    short pullDir = 0;  // 0 is no change, 1 is up 2 is down
    std::string topic;

    // comparison operators needed for mapping
    friend bool operator<(const GpioPort& l, const GpioPort& r) {
        return std::tie(l.chip, l.port) < std::tie(r.chip, r.port);
    }
    friend bool operator>(const GpioPort& lhs, const GpioPort& rhs) { return rhs < lhs; }
    friend bool operator<=(const GpioPort& lhs, const GpioPort& rhs) { return !(lhs > rhs); }
    friend bool operator>=(const GpioPort& lhs, const GpioPort& rhs) { return !(lhs < rhs); }
};

std::shared_ptr<std::vector<GpioPort>> createGpioMap(TiXmlDocument* doc);

class LineCaller {
private:
    std::shared_ptr<gpiod::chip> gpioChip;
    std::shared_ptr<gpiod::line> line;
    std::shared_ptr<short> pinDir;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr pubTimer;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
    bool unsafeFlag;

public:
    LineCaller(std::string& chip, int port, short pullDir, short pinDirConf, rclcpp::Node& node, const std::string& topic) {
        gpioChip = std::make_shared<gpiod::chip>(chip, 1);
        line = std::make_shared<gpiod::line>(gpioChip->get_line(port));
        pinDir = std::make_shared<short>(pinDirConf);

        int dir = 0;
        switch (*pinDir) {
        case 0:
            dir = gpiod::line_request::DIRECTION_INPUT;
            pub = node.create_publisher<std_msgs::msg::Bool>(topic, 10);
            pubTimer = node.create_wall_timer(10ms, std::bind(&LineCaller::readCallback, this));
            break;
        case 1:
            dir = gpiod::line_request::DIRECTION_OUTPUT;
            sub = node.create_subscription<std_msgs::msg::Bool>(topic, 10, std::bind(&LineCaller::updateCallback, this, _1));
            break;
        case 2:
            dir = gpiod::line_request::DIRECTION_OUTPUT;
            break;
        default:
            dir = gpiod::line_request::DIRECTION_AS_IS;
        }

        std::bitset<32UL> flags = 0;
#ifdef ALLOW_PULL_DIR
        switch (pullDir) {
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
#endif

        line->request({"hw_interface", dir, flags});
    }

    short getDir() {
        return *(pinDir);
    }

    void setVal(bool val) {
        if (*pinDir > 0) {
            if (unsafeFlag)
            std::cout << "setting line to " << val << std::endl;
                line->set_value(val);
            else
                line->set_value(false);
        }
    }

    void updateCallback(std::shared_ptr<std_msgs::msg::Bool> msg) {
        setVal(msg->data);
    }

    void readCallback() {
        if (*pinDir == 0) {
            std_msgs::msg::Bool msg = std_msgs::msg::Bool();
            msg.data = line->get_value();
            pub->publish(msg);
        }
    }

    void setSafeFlag(bool flag) {
        unsafeFlag = flag;
    }
};

class HardwareController : public rclcpp::Node {
private:
    //map of all gpio lines requested by system
    std::map<GpioPort, std::shared_ptr<LineCaller>> lines;

    //list of all subscribed topics
    std::vector<std::string> topics;

    //safety enable subscription that allows motors to be active
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;

    //safety enable timer
    rclcpp::TimerBase::SharedPtr safetyTimer;

    //flag for saftey enable of all outputs
    bool safetyEnable = false;

    //last time the safety stamp was set to true
    std::chrono::_V2::system_clock::duration safetyStamp;

    bool safetyToggle = false;

public:
    HardwareController() : Node("pi_hw_interface") {
        safetySubscrip = create_subscription<std_msgs::msg::Bool>("safety_enable", 10, std::bind(&HardwareController::feedSafety, this, _1));
        safetyStamp = std::chrono::system_clock::now().time_since_epoch();
        safetyTimer = create_wall_timer(SAFETY_TIMEOUT_CHECK, std::bind(&HardwareController::safetyTimerUpdate, this));
    }

    void registerGpio(std::vector<GpioPort> ports) {
        try {
            //create all GPIO
            for (auto it = ports.begin(); it != ports.end(); it++) {
                //RCLCPP_INFO(this->get_logger(), "Creating LineCaller %s %d %d %d", it->chip.c_str(), it->port, it->pullDir, it->pinDir);

                std::shared_ptr<LineCaller> line = std::make_shared<LineCaller>(it->chip, it->port, it->pullDir, it->pinDir, (*this), it->topic);

                lines[*it] = line;
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
            unsafe();
        }
    }

    void safetyTimerUpdate() {
        //RCLCPP_INFO(this->get_logger(), "Safety timer tick");
        auto now = std::chrono::system_clock::now().time_since_epoch();
        if (safetyEnable && (now - SAFETY_TIMEOUT > safetyStamp)) {
            RCLCPP_INFO(this->get_logger(), "Safety timer expired");
            safe();
        }

        //if (safetyEnable && (now - (SAFETY_TIMEOUT / 2) > safetyStamp))
        //    safetyToggle = !safetyToggle;
    }

    void safe() {
        safetyEnable = false;
        RCLCPP_INFO(this->get_logger(), "Safing lines");
        std::map<GpioPort, std::shared_ptr<LineCaller>>::iterator it;
        for (it = lines.begin(); it != lines.end(); it++) {
            if (it->second->getDir() > 0) {
                it->second->setSafeFlag(false);
            }
        }
    }

    void unsafe() {
        if (!safetyEnable) {
            RCLCPP_INFO(this->get_logger(), "Unsafing lines");
            safetyEnable = true;
            std::map<GpioPort, std::shared_ptr<LineCaller>>::iterator it;
            for (it = lines.begin(); it != lines.end(); it++) {
                if (it->second->getDir() > 0) {
                    it->second->setSafeFlag(true);
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    //init ros node
    rclcpp::init(argc, argv);
    std::shared_ptr<HardwareController> rosNode = std::make_shared<HardwareController>();
    RCLCPP_INFO(rosNode->get_logger(), "Hardware interface node starting");

    // Load the xml file
    std::filesystem::path config = std::filesystem::current_path() / "config.xml";
    TiXmlDocument* doc = new TiXmlDocument(config.c_str());
    if (!doc->LoadFile()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Error parsing XML config %s\n %s", config.c_str(), doc->ErrorDesc());
    } else {
        try {
            // grab the parent motor XML element and make sure it exists
            TiXmlElement* hardware = doc->FirstChildElement("hardware");
            if (!hardware) throw std::runtime_error("XML doc is missing root hardware element");
            TiXmlElement* gpios = hardware->FirstChildElement("gpios");
            if (!gpios) throw std::runtime_error("XML doc is missing gpios element. The gpios element should be defined even if there are no gpio being created");

            // RCLCPP_INFO(rosNode->get_logger(), "XML doc loaded, outer parts intact");
            std::vector<GpioPort> gpioList = std::vector<GpioPort>();

            for (TiXmlElement* gpio = gpios->FirstChildElement("gpio"); gpio != nullptr; gpio = gpio->NextSiblingElement("gpio")) {
                GpioPort port = {};
                std::string tmp, tmp1, topic;
                if (!getValue(gpio, "topic", topic)) {
                    throw std::runtime_error("Gpio definition missing topic name");
                }
                if (!getValue(gpio, "port", tmp)) {
                    throw std::runtime_error("Gpio definition missing port number");
                }
                if (!getValue(gpio, "dir", tmp1)) {
                    throw std::runtime_error("Gpio definition missing dir");
                }

                port.port = std::stoi(tmp);
                if (tmp1 == "OUT")
                    port.pinDir = 1;
                else if (tmp1 == "IN")
                    port.pinDir = 0;
                else
                    port.pinDir = 3;

                if (getValue(gpio, "pull", tmp)) port.pullDir = std::stoi(tmp);

                port.topic = "pi_hw_interface/" + topic;

                gpioList.push_back(port);

                RCLCPP_INFO(rosNode->get_logger(), "Got line config Topic: %s  Port: %s  Dir: %s", port.topic.c_str(), tmp.c_str(), tmp1.c_str());
            }

            RCLCPP_INFO(rosNode->get_logger(), "Recieved config for %d gpio(s)", gpioList.size());
            rosNode->registerGpio(gpioList);

            //RCLCPP_INFO(rosNode->get_logger(), "Registered GPIO Lines");

            //set all gpio to off
            rosNode->safe();

            RCLCPP_INFO(rosNode->get_logger(), "Hardware interface node loaded using gpiod interface");

            // serve the callbacks
            rclcpp::spin(rosNode);

            RCLCPP_INFO(rosNode->get_logger(), "Hardware interface shutting down");

            //set all gpio to off
            rosNode->safe();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rosNode->get_logger(), "Node failed\nCause: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(rosNode->get_logger(), "Node failed\nCause Unknown");
        }
    }

    delete doc;

    RCLCPP_INFO(rosNode->get_logger(), "Hardware interface shut down complete");

    rclcpp::shutdown();

    return 0;
}

/**
 * function to pull a child element from a parent element as text
 * @param elem the parent XML element
 * @param childName the name of the XML child element to find
 * @param value the resulting value of the child element.
 * @return bool true if the element was found or false if not found
 **/
bool getValue(TiXmlElement* elem, const std::string& childName, std::string& value) {
    //std::cout << "checking for " << childName << std::endl;

    //make sure child element and corresponding text exists
    TiXmlElement* childElem = elem->FirstChildElement(childName);
    if (!childElem) return false;

    const char* xmlVal = childElem->GetText();
    if (!xmlVal) return false;

    value = std::string(xmlVal);
    return true;
}

/**
 * @param s the string to parse for a boolean
 * @return the value of the resulting boolean
 **/
bool stob(std::string s) {
    auto result = false;  // failure to assert is false

    std::istringstream is(s);
    // first try simple integer conversion
    is >> result;

    if (is.fail()) {
        // simple integer failed; try boolean
        is.clear();
        is >> std::boolalpha >> result;
    }
    return result;
}