#include <iostream>

#include "SerialPort.hpp"

int main(int argc, char **argv)
{
    //rclcpp::init(argc, argv);
    //rclcpp::shutdown();
    SerialPort serial("/dev/ttyACM0");
    while(1){}
    return 0;
}