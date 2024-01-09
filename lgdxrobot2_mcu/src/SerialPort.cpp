#include <filesystem>
#include <chrono>
#include <iostream>

#include "SerialPort.hpp"

void SerialPort::autoSearch()
{
  int wait = 3;
  bool found = false;
  std::string port;
  while(!found)
  {
    std::filesystem::path path {"/dev"};
    for(auto const &file : std::filesystem::directory_iterator(path))
    {
      if(file.path().string().find("ttyACM") != std::string::npos)
      {
        port = file.path().string();
        std::string msg = std::string("Serial device ") + port + std::string(" found.");
        debug(msg, 1);
        found = true;
        break;
      }  
    }
    if(!found)
    {
      std::string msg = std::string("No serial device found, wait for ") + std::to_string(wait) + std::string(" seconds.");
      debug(msg, 1);
      std::this_thread::sleep_for(std::chrono::seconds(wait));
    }
  }
  connect(port);
}

void SerialPort::reconnect()
{
  serial.close();
  autoSearch();
}

void SerialPort::read()
{
  serial.async_read_some(boost::asio::buffer(readBuffer, kReadBufferSize), std::bind(&SerialPort::readHandler, this, std::placeholders::_1, std::placeholders::_2));
}

void SerialPort::readHandler(boost::system::error_code error, std::size_t size)
{
  if(!error)
  {
    if(size >= 5) 
    {
      // Find the header and frame size
      if(readBuffer[0] == char(170)) // 170 = 0xAA
      {
        localReadBufferTargetSize = combineBytes((uint8_t) readBuffer[1], (uint8_t) readBuffer[2], (uint8_t) readBuffer[3], (uint8_t) readBuffer[4]);
        if(int(size) == localReadBufferTargetSize)
        {
          // Process the data if received data = target size, 
          processReadData();
        }
      }
    }

    // Read upcoming data
    read();
  }
  else 
  {
    std::string msg = std::string("Serial read throws an error: ") + std::string(error.message());
    debug(msg, 3);
    reconnect();
  }
}

void SerialPort::processReadData()
{
  int index = 5;
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    mcuData.targetWheelVelocity[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    mcuData.measuredWheelVelocity[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    mcuData.pConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    mcuData.iConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    mcuData.dConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < 2; i++) 
  {
    mcuData.battery[i] = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    mcuData.battery[i] *= 0.004;
    index += 4;
  }
  for(int i = 0; i < 2; i++) 
  {
    mcuData.estop[i] = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    index += 4;
  }
  if(readCallback)
  {
    readCallback(mcuData);
  }
}

void SerialPort::write(std::vector<char> &data)
{
 serial.async_write_some(boost::asio::buffer(data), std::bind(&SerialPort::readHandler, this, std::placeholders::_1, std::placeholders::_2));
}

void SerialPort::writeHandler(boost::system::error_code error, std::size_t size)
{
  if(error) 
  {
    std::string msg = std::string("Serial write throws an error: ") + std::string(error.message());
    debug(msg, 3);
    reconnect();
  }
}

void SerialPort::debug(const std::string &msg, int level)
{
  if(debugCallback)
    debugCallback(msg, level);
}

SerialPort::SerialPort(std::function<void(const McuData &)> read, std::function<void(const std::string&, int)> debug) : ioservice(), serial(ioservice), readCallback(read), debugCallback(debug)
{
  autoSearch();
}

SerialPort::~SerialPort()
{
  serial.close();
  ioservice.stop();
  ioThread.join();
}

void SerialPort::connect(std::string &port)
{
  serial.open(port);
  if(!firstConnection) 
  {
    std::thread thread{[this](){ ioservice.run(); }};
    ioThread.swap(thread);
    firstConnection = true;
  }
  read();
}

void SerialPort::setInverseKinematics(float x, float y, float w)
{
  uint32_t ux = floatToUint32(x);
  uint32_t uy = floatToUint32(y);
  uint32_t uw = floatToUint32(w);
  std::vector<char> ba(13);
  ba[0] = 'M';
  ba[1] = (ux & 4278190080) >> 24;
  ba[2] = (ux & 16711680) >> 16;
  ba[3] = (ux & 65280) >> 8;
  ba[4] = ux & 255;
  ba[5] = (uy & 4278190080) >> 24;
  ba[6] = (uy & 16711680) >> 16;
  ba[7] = (uy & 65280) >> 8;
  ba[8] = uy & 255;
  ba[9] = (uw & 4278190080) >> 24;
  ba[10] = (uw & 16711680) >> 16;
  ba[11] = (uw & 65280) >> 8;
  ba[12] = uw & 255;
  write(ba);
}

void SerialPort::setEstop(int enable)
{
  std::vector<char> ba(5);
  ba[0] = 'E';
  ba[1] = (enable & 4278190080) >> 24;
  ba[2] = (enable & 16711680) >> 16;
  ba[3] = (enable & 65280) >> 8;
  ba[4] = enable & 255;
  write(ba);
}