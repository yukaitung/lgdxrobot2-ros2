#include <chrono>
#include <cstdio>
#include <filesystem>

#include "lgdxrobot2_daemon/SerialPort.hpp"

SerialPort::SerialPort(const std::string port,
  bool resetTransform,
  std::function<void(const RobotData &)> updateDaemonCb,
  std::function<void(const char *, int)> logCb) : 
    serialService(), 
    serial(serialService), 
    timerService(), 
    timer(timerService)
{
  resetTransformOnConnected = resetTransform;
  updateDeamon = updateDaemonCb;
  log = logCb;
  if(port.empty())
  {
    // Perform auto search if no port specified
    autoSearch();
  }
  else
  {
    defaultPortName = port;
    connect(defaultPortName);
  }
}

SerialPort::~SerialPort()
{
  timerService.stop();
  if(timerThread.joinable())
    timerThread.join();
  timer.cancel();
  serialService.stop();
  if(ioThread.joinable())
    ioThread.join();
  serial.close();
}

void SerialPort::startSerialIo()
{
  if(ioThread.joinable()) 
  {
    serialService.restart();
    ioThread.join();
  }
  std::thread thread{[this](){ serialService.run(); }};
  ioThread.swap(thread);
}

void SerialPort::startTimerIo()
{
  if(!timerService.stopped() && timerThread.joinable()) 
  {
    // The timer is continue running, don't do anything
    return;
  }
  if(timerThread.joinable())
  {
    // timerService returned, restart the timerService
    timerService.restart();
    timerThread.join();    
  }
  std::thread thread{[this](){ timerService.run(); }};
  timerThread.swap(thread);
}

void SerialPort::autoSearch()
{
  char msg[100];
  std::string port;
  std::filesystem::path path {"/dev"};
  for(auto const &file : std::filesystem::directory_iterator(path))
  {
    // Linux only, find first /dev/ttyACM*
    if(file.path().string().find("ttyACM") != std::string::npos)
    {
      port = file.path().string();
      sprintf(msg, "Serial device %s found.", port.c_str());
      log(msg, 1);
      connect(port);
      return;
    }  
  }
  sprintf(msg, "No serial device found, try again in %s seconds.", port.c_str());
  log(msg, 1);
  timer.expires_after(std::chrono::seconds(kWaitSecond));
  timer.async_wait(std::bind(&SerialPort::autoSearch, this));
  startTimerIo();
}

void SerialPort::connect(const std::string &port)
{
  char msg[250];
  boost::system::error_code error;
  serial.open(port, error);
  if(error) 
  {
    sprintf(msg, "Serial connection throws an error: %s, try again in %d seconds.", error.message().c_str(), kWaitSecond);
    log(msg, 3);
    timer.expires_after(std::chrono::seconds(kWaitSecond));
    timer.async_wait(std::bind(&SerialPort::connect, this, port));
    startTimerIo();
    return;
  }
  sprintf(msg, "Serial port connected to %s", port.c_str());
  log(msg, 1);
  read();
  if(resetTransformOnConnected)
  {
    resetTransformOnConnected = false;
    resetTransformInternal();
  }
  startSerialIo();
}

void SerialPort::reconnect()
{
  if(defaultPortName.empty()) // Perform auto search if no defaultPortName
    autoSearch();
  else
    connect(defaultPortName);
}

void SerialPort::read()
{
  serial.async_read_some(boost::asio::buffer(readBuffer, kReadBufferSize), std::bind(&SerialPort::readHandler, this, std::placeholders::_1, std::placeholders::_2));    
}

void SerialPort::readHandler(boost::system::error_code error, std::size_t size)
{
  if(!serial.is_open())
    return; // Discard further error is the serial is closed
  if(!error)
  {
    if(size >= 2) 
    {
      // Find the header and frame size
      if(readBuffer[0] == char(170)) // 170 = 0xAA
      {
        int frameSize = readBuffer[1];
        if(int(size) == frameSize)
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
    char msg[100];
    sprintf(msg, "Serial read throws an error: %s", error.message().c_str());
    log(msg, 3);
    //serialService.stop();
    serial.close();
    reconnect();
  }
}

void SerialPort::processReadData()
{
  int index = 2;
  robotData.refreshTime = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1]);
  index += 2;
  for(int i = 0; i < 3; i++)
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.transform[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < 3; i++)
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.forwardKinematic[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.targetWheelVelocity[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.measuredWheelVelocity[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.pConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.iConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.dConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < 2; i++) 
  {
    robotData.battery[i] = combineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1]) * 0.004;
    index += 2;
  }
  uint8_t eStopByte = readBuffer[index];
  int compare = 128;
  for(int i = 0; i < 2; i++)
  {
    robotData.eStop[i] = (eStopByte & compare) >> (7 - i);
    compare = compare >> 1;
  }
  updateDeamon(robotData);
}

void SerialPort::resetTransformInternal()
{
  std::vector<char> ba(1);
  ba[0] = 'T';
  write(ba);
}

void SerialPort::write(const std::vector<char> &data)
{
  if(serial.is_open())
    serial.async_write_some(boost::asio::buffer(data), std::bind(&SerialPort::readHandler, this, std::placeholders::_1, 0));
}

void SerialPort::writeHandler(boost::system::error_code error)
{ 
  if(error) 
  {
    char msg[100];
    sprintf(msg, "Serial read throws an error: %s", error.message().c_str());
    log(msg, 3);
  }
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

void SerialPort::resetTransform()
{
  if(serial.is_open())
    resetTransformInternal();
  else
    resetTransformOnConnected = true;
}

void SerialPort::setExternalImu(float ax, float ay, float az, float gz)
{
  uint32_t ux = floatToUint32(ax);
  uint32_t uy = floatToUint32(ay);
  uint32_t uz = floatToUint32(az);
  uint32_t ugz = floatToUint32(gz);
  std::vector<char> ba(17);
  ba[0] = 'M';
  ba[1] = (ux & 4278190080) >> 24;
  ba[2] = (ux & 16711680) >> 16;
  ba[3] = (ux & 65280) >> 8;
  ba[4] = ux & 255;
  ba[5] = (uy & 4278190080) >> 24;
  ba[6] = (uy & 16711680) >> 16;
  ba[7] = (uy & 65280) >> 8;
  ba[8] = uy & 255;
  ba[9] = (uz & 4278190080) >> 24;
  ba[10] = (uz & 16711680) >> 16;
  ba[11] = (uz & 65280) >> 8;
  ba[12] = uz & 255;
  ba[13] = (ugz & 4278190080) >> 24;
  ba[14] = (ugz & 16711680) >> 16;
  ba[15] = (ugz & 65280) >> 8;
  ba[16] = ugz & 255;
  write(ba);
}