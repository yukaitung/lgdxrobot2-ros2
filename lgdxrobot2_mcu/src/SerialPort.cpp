#include <iostream>
#include "SerialPort.hpp"

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
  for(int i = 0; i < 4; i++) {
      uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      float a = uint32ToFloat(temp);
      std::cout << "Target Wheels Velocity " << i + 1 << " " << a << std::endl;
      index += 4;
  }
  for(int i = 0; i < 4; i++) {
      uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      float a = uint32ToFloat(temp);
      std::cout << "Measured Wheels Velocity " << i + 1 << " " << a << std::endl;
      index += 4;
  }
  for(int i = 0; i < 4; i++) {
      uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      float a  = uint32ToFloat(temp);
      std::cout << "P Constant " << i + 1 << " " << a << std::endl;
      index += 4;
  }
  for(int i = 0; i < 4; i++) {
      uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      float a  = uint32ToFloat(temp);
      std::cout << "I Constant " << i + 1 << " " << a << std::endl;
      index += 4;
  }
  for(int i = 0; i < 4; i++) {
      uint32_t temp = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      float a  = uint32ToFloat(temp);
      std::cout << "D Constant " << i + 1 << " " << a << std::endl;
      index += 4;
  }
  for(int i = 0; i < 4; i++) {
      int a  = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      std::cout << "PWM  " << i + 1 << " " << a << std::endl;
      index += 4;
  }
  for(int i = 0; i < 2; i++) {
      int a  = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      std::cout << "Battery  " << i + 1 << " " << a*0.004 << std::endl;
      index += 4;
  }
  for(int i = 0; i < 2; i++) {
      int a = combineBytes((uint8_t) data[index], (uint8_t) data[index + 1], (uint8_t) data[index + 2], (uint8_t) data[index + 3]);
      std::cout << "Hardware E-Stop Enabled " << i + 1 << " " << a << std::endl;
      index += 4;
  }
}