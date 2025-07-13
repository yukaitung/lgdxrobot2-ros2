#ifndef MCU_HPP
#define MCU_HPP

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "Structs/RobotData.hpp"

class Mcu
{
  private:
    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr serialPortReconnectTimer;

    boost::asio::io_service serialService;
    boost::asio::serial_port serial;
    std::thread ioThread;

    RobotData robotData;
    std::string portName;
    int kWaitSecond = 3;
    bool resetTransformOnConnected = false;
    static const int kReadBufferSize = 2048;
    char readBuffer[kReadBufferSize] = {0};

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
    void ProcessRobotData();
    void CharArrayToHex(const char* input, size_t length, char* output);
    void GetSerialNumber();
    void ProcessSerialNumber();

    // Write to MCU
    void ResetTransformInternal();
    void Write(const std::vector<char> &data);
    void OnWriteComplete(boost::system::error_code error);

  public:
    Mcu(rclcpp::Node::SharedPtr node);
    ~Mcu();

    void SetInverseKinematics(float x, float y, float w);
    void SetEstop(int enable);
    void ResetTransform();
    void SetExternalImu(float ax, float ay, float az, float gz);
};

#endif // MCU_HPP