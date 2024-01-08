#include <memory>

#include "McuNode.hpp"
#include "SerialPort.hpp"

int main(int argc, char **argv)
{
    auto serial = std::make_shared<SerialPort>("/dev/ttyACM0");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<McuNode>(serial));
    rclcpp::shutdown();
    return 0;
}