#ifndef MCU_HPP
#define MCU_HPP

#include <array>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"

#include "lgdxrobot2.h"
#include "McuSignals.hpp"

class Mcu
{
  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Logger _logger;
    rclcpp::TimerBase::SharedPtr serialPortReconnectTimer;

    boost::asio::io_service serialService;
    boost::asio::serial_port serial;
    std::thread ioThread;

    std::shared_ptr<McuSignals> mcuSignals;

    // Constants
    int kWaitSecond = 3;

    // Settings
    bool resetTransformOnConnected = false;

    // Data
    bool hasSerialNumber = false;
    McuData mcuData;
    std::array<uint8_t, 512> readBuffer = {0};
    std::vector<uint8_t> mcuBuffer = {0};

    // io_service thread
    void StartSerialIo();

    // Connection
    void Connect();

    // Read from MCU
    void Read();
    void OnReadComplete(boost::system::error_code error, std::size_t size);
    std::string SerialToHexString(uint32_t serial1, uint32_t serial2, uint32_t serial3);
    void ProcessSerialNumber(const McuSerialNumber &mcuSerialNumber);

    // Write to MCU
    void ResetTransformInternal();
    void Write(const std::vector<char> &data);
    void OnWriteComplete(boost::system::error_code error);

  public:
    Mcu(rclcpp::Node::SharedPtr node, std::shared_ptr<McuSignals> mcuSignalsPtr);
    ~Mcu();

    void SetInverseKinematics(float x, float y, float w);
    void SetEstop(int enable);
    void ResetTransform();
    void GetSerialNumber();
};

#endif // MCU_HPP