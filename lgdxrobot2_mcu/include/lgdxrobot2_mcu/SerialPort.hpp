#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <string>
#include <thread>
#include <functional>
#include <boost/asio.hpp>

#include "McuData.hpp"

class SerialPort
{
  private:
    // Objects
    std::string portName;
    boost::asio::io_service ioservice;
    boost::asio::serial_port serial;
    std::thread ioThread;
    McuData mcuData;
    std::function<void(McuData const&)> readCallback = nullptr;

    // Util
    float uint32ToFloat(uint32_t n){ return (float)(*(float*)&n); }
    uint32_t combineBytes(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
      return a << 24 | b << 16 | c << 8 | d;
    }

    // Read Buffer
    static const int kReadBufferSize = 512;
    char readBuffer[kReadBufferSize] = {0};
    char localReadBuffer[kReadBufferSize] = {0}; // Read buffer for longer storage
    int localReadBufferCount = 0;
    int localReadBufferTargetSize = 0;

    // Write Buffer

    // Read from MCU
    void read();
    void readHandler(boost::system::error_code error, std::size_t size);
    void saveReadBuffer(int size);
    void clearReadBuffer();
    void processReadData(char* const data);
  
  public:
    SerialPort(std::string portName);
    ~SerialPort();

    void setReadCallback(std::function<void(McuData const&)> f);
};

#endif // SERIALPORT_HPP