#ifndef MCU_HPP
#define MCU_HPP

#include <array>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"

#include "lgdxrobot2.h"
#include "McuSignals.hpp"

using boost::asio::awaitable;

class Mcu
{
  private:
    rclcpp::Logger _logger;
    rclcpp::TimerBase::SharedPtr serialPortReconnectTimer;

    boost::asio::io_context ioContext;
    boost::asio::serial_port serial;
    std::thread ioThread;

    std::shared_ptr<McuSignals> mcuSignals;

    // Constants
    int kWaitSecond = 3;

    // Settings
    std::string serialPortName;
    bool resetTransformOnConnected = false;
    bool isShuttingDown = false;

    // Data
    McuData mcuData;
    std::array<uint8_t, 512> readBuffer = {0};
    std::vector<uint8_t> mcuBuffer = {0};

    // io_context thread
    void StartSerialIo();

    // Connection
    void Connect();

    // Read from MCU
    awaitable<void> Read();
    void OnReadComplete(size_t size);

    // Write to MCU
    void ResetTransformInternal();
    awaitable<void> Write(const std::vector<char> data);

  public:
    Mcu(rclcpp::Node::SharedPtr node, std::shared_ptr<McuSignals> mcuSignalsPtr);
    ~Mcu();

    void SetInverseKinematics(float x, float y, float w);
    void SetEstop(int enable);
    void ResetTransform();
};

#endif // MCU_HPP