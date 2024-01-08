#include "SerialPort.hpp"

void SerialPort::read()
{
  serial.async_read_some(boost::asio::buffer(serialBuffer, kSerialBufferSize), std::bind(&SerialPort::readHandler, this, std::placeholders::_1, std::placeholders::_2));
}

void SerialPort::readHandler(boost::system::error_code error, std::size_t size)
{
  if(!error)
  {
    if(size >= 2) 
    {
      // Find the header and frame size
      if(serialBuffer[0] == char(170)) // 170 = 0xABAB
      {
        frameBufferTargetSize = int(serialBuffer[1]);
        if(int(size) == frameBufferTargetSize)
        {
          // Process the data if received data = target size, 
          processSerialData(serialBuffer);
        }
        else if(int(size) < frameBufferTargetSize)
        {
          // If the received data is too few, copy them into framebuffer
          saveSerialBuffer(size);
        }
        // If the received data is too many, discard the data
      }
    }
    
    if(frameBufferCount > 0)
    {
      // When using framebuffer, save more data
      saveSerialBuffer(size);
      if(frameBufferCount == frameBufferTargetSize)
      {
        processSerialData(framebuffer);
        clearSerialBuffer();
      }
      if(frameBufferCount > frameBufferTargetSize)
      {
        // Discard the data if too much
        clearSerialBuffer();
      }
    }
    
    // Read upcoming data
    read();
  }
}

void SerialPort::saveSerialBuffer(int size)
{
  for(int i = 0; i < size; i++)
  {
    if(i > kSerialBufferSize)
      return;
    framebuffer[frameBufferCount + i] = serialBuffer[i];
  }
  frameBufferCount += size;
}

void SerialPort::clearSerialBuffer()
{
  frameBufferCount = 0;
}

void SerialPort::processSerialData(char* const data)
{
  int index = 2;
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    mcuData.targetWheelVelocity[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    mcuData.measuredWheelVelocity[i] = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    mcuData.pConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    mcuData.iConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++) 
  {
    uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    mcuData.dConstant[i]  = uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < mcuData.wheelCount; i++)
  {
    mcuData.pwm[i] = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    index += 4;
  }
  for(int i = 0; i < 2; i++) 
  {
    mcuData.battery[i] = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    mcuData.battery[i] *= 0.004;
    index += 4;
  }
  for(int i = 0; i < 2; i++) 
  {
    mcuData.estop[i] = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
    index += 4;
  }
  if(readCallback)
  {
    readCallback(mcuData);
  }
}

SerialPort::SerialPort(std::string portName) : ioservice(), serial(ioservice)
{
  serial.open(portName);
  std::thread thread{[this](){ ioservice.run(); }};
  ioThread.swap(thread);
  read();
}

SerialPort::~SerialPort()
{
  ioservice.stop();
  ioThread.join();
}

void SerialPort::setReadCallback(std::function<void(McuData const&)> f)
{
  readCallback = f;
}