#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <string>
#include <vector>
#include <thread>
#include <functional>
#include <boost/asio.hpp>

#include "RobotData.hpp"

class SerialPort
{
  private:
    const int kWaitSecond = 3;
    std::string defaultPortName;
    bool resetTransformOnConnected = false;
    std::function<void(const RobotData &)> updateDeamon;
    std::function<void(const char *, int)> log;
    std::function<void(const char *)> serialNumber;

    RobotData robotData;
    boost::asio::io_service serialService;
    boost::asio::serial_port serial;
    std::thread ioThread;
    boost::asio::io_service timerService;
    boost::asio::steady_timer timer;
    std::thread timerThread;

    // Read Buffer
    static const int kReadBufferSize = 2048;
    char readBuffer[kReadBufferSize] = {0};

    // Util
    inline uint32_t floatToUint32(float n){ return (uint32_t)(*(uint32_t*)&n); }
    inline float uint32ToFloat(uint32_t n){ return (float)(*(float*)&n); }
    inline uint32_t combineBytes(uint32_t a, uint32_t b, uint32_t c, uint32_t d) { return a << 24 | b << 16 | c << 8 | d; }
    inline uint16_t combineBytes(uint16_t a, uint16_t b) { return a << 8 | b; }

    // io_service thread
    void startSerialIo();
    void startTimerIo();
    
    // Connection
    void autoSearch();
    void connect(const std::string &port);
    void reconnect();

    // Read from MCU
    void read();
    void readHandler(boost::system::error_code error, std::size_t size);
    void processReadData();
    void charArrayToHex(const char* input, size_t length, char* output);
    void getSerialNumber();
    void processSerialNumber();

    // Write to MCU
    void resetTransformInternal();
    void write(const std::vector<char> &data);
    void writeHandler(boost::system::error_code error);
  
  public:
    SerialPort(const std::string port, // Pass by value
      bool resetTransform,
      std::function<void(const RobotData &)> updateDaemonCb,
      std::function<void(const char *, int)> logCb,
      std::function<void(const char *)> serialNumberCb);
    ~SerialPort();

    void start(const std::string &port);
    void setInverseKinematics(float x, float y, float w);
    void setEstop(int enable);
    void resetTransform();
    void setExternalImu(float ax, float ay, float az, float gz);
};

#endif // SERIALPORT_HPP