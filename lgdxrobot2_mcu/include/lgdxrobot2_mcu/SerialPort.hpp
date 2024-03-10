#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <string>
#include <vector>
#include <thread>
#include <functional>
#include <boost/asio.hpp>

#include "McuData.hpp"

class SerialPort
{
  private:
    const int kWaitSecond = 3;
    std::string defaultPortName;
    boost::asio::io_service serialService;
    boost::asio::serial_port serial;
    std::thread ioThread;
    boost::asio::io_service timerService;
    boost::asio::steady_timer timer;
    std::thread timerThread;
    McuData mcuData;
    std::function<void(const McuData &)> readCallback = nullptr;
    std::function<void(const std::string &, int)> debugCallback = nullptr;
    bool resetTransformOnConnected = false;

    // Read Buffer
    static const int kReadBufferSize = 2048;
    char readBuffer[kReadBufferSize] = {0};

    // Util
    uint32_t floatToUint32(float n){ return (uint32_t)(*(uint32_t*)&n); }
    float uint32ToFloat(uint32_t n){ return (float)(*(float*)&n); }
    uint32_t combineBytes(uint32_t a, uint32_t b, uint32_t c, uint32_t d) { return a << 24 | b << 16 | c << 8 | d; }
    uint16_t combineBytes(uint16_t a, uint16_t b) { return a << 8 | b; }

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

    // Write to MCU
    void resetTransformPrivate();
    void write(const std::vector<char> &data);
    void writeHandler(boost::system::error_code error);

    void debug(const std::string &msg, int level);
  
  public:
    SerialPort(std::function<void(const McuData &)> read, std::function<void(const std::string&, int)> debug);
    ~SerialPort();

    void start(const std::string &port);
    void setInverseKinematics(float x, float y, float w);
    void setEstop(int enable);
    void resetTransform();
    void setExternalImu(float ax, float ay, float az, float gz);
};

#endif // SERIALPORT_HPP