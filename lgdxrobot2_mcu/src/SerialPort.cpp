#include "SerialPort.hpp"

void SerialPort::read()
{
  serial.async_read_some(boost::asio::buffer(readBuffer, kReadBufferSize), std::bind(&SerialPort::readHandler, this, std::placeholders::_1, std::placeholders::_2));
}

void SerialPort::readHandler(boost::system::error_code error, std::size_t size)
{
  if(!error)
  {
    if(size >= 2) 
    {
      // Find the header and frame size
      if(readBuffer[0] == char(170)) // 170 = 0xABAB
      {
        localReadBufferTargetSize = int(readBuffer[1]);
        if(int(size) == localReadBufferTargetSize)
        {
          // Process the data if received data = target size, 
          processReadData(readBuffer);
        }
        else if(int(size) < localReadBufferTargetSize)
        {
          // If the received data is too few, copy them into localReadBuffer
          saveReadBuffer(size);
        }
        // If the received data is too many, discard the data
      }
    }
    
    if(localReadBufferCount > 0)
    {
      // When using localReadBuffer, save more data
      saveReadBuffer(size);
      if(localReadBufferCount == localReadBufferTargetSize)
      {
        processReadData(localReadBuffer);
        clearReadBuffer();
      }
      if(localReadBufferCount > localReadBufferTargetSize)
      {
        // Discard the data if too much
        clearReadBuffer();
      }
    }
    
    // Read upcoming data
    read();
  }
}

void SerialPort::saveReadBuffer(int size)
{
  for(int i = 0; i < size; i++)
  {
    if(i >= kReadBufferSize)
      return;
    localReadBuffer[localReadBufferCount + i] = readBuffer[i];
  }
  localReadBufferCount += size;
}

void SerialPort::clearReadBuffer()
{
  localReadBufferCount = 0;
}

void SerialPort::processReadData(char* const data)
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

void SerialPort::write(std::vector<char> &data)
{
 serial.async_write_some(boost::asio::buffer(data), std::bind(&SerialPort::readHandler, this, std::placeholders::_1, std::placeholders::_2));
}

void SerialPort::writeHandler(boost::system::error_code error, std::size_t size)
{

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