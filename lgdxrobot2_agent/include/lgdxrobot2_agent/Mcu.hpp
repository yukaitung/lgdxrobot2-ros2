#ifndef MCU_HPP
#define MCU_HPP

#include <array>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "Structs/McuSignals.hpp"
#include "Structs/RobotData.hpp"
#include "lgdxrobot2_agent/lgdxrobot2.h"

class Mcu
{
  private:
    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr serialPortReconnectTimer;

    boost::asio::io_service serialService;
    boost::asio::serial_port serial;
    std::thread ioThread;

    std::shared_ptr<McuSignals> mcuSignals;

    // Constants
    int kWaitSecond = 3;

    // Settings
    std::string portName;
    bool resetTransformOnConnected = false;

    // Data
    RobotData robotData;
    std::array<uint8_t, 512> readBuffer = {0};
    std::vector<uint8_t> mcuBuffer = {0};

    // Util
    inline uint32_t FloatToUint32(float n){ return (uint32_t)(*(uint32_t*)&n); }
    inline float Uint32ToFloat(uint32_t n){ return (float)(*(float*)&n); }
    inline uint32_t CombineBytes(uint32_t a, uint32_t b, uint32_t c, uint32_t d) { return a << 24 | b << 16 | c << 8 | d; }
    inline uint16_t CombineBytes(uint16_t a, uint16_t b) { return a << 8 | b; }

    // io_service thread
    void StartSerialIo();

    // Connection
    void AutoSearch();
    void Connect(const std::string &port);
    void Reconnect();

    // Read from MCU
    void Read();
    void OnReadComplete(boost::system::error_code error, std::size_t size);
    void ProcessRobotData(const McuData &mcuData);
    std::string SerialToHexString(uint32_t serial1, uint32_t serial2, uint32_t serial3);
    void GetSerialNumber();
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
};

#endif // MCU_HPP