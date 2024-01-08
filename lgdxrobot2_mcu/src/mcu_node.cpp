#include <iostream>
#include <boost/multiprecision/cpp_int.hpp>

#include "rclcpp/rclcpp.hpp"

using boost::multiprecision::cpp_int;

int main(int argc, char **argv)
{
    cpp_int n = 1844674407370955161;
    cpp_int p = 1844674407370955161;
    std::cout << n * p << std::endl;
    return 0;
}