#include <filesystem>

#include "lgdxrobot2_agent/Mcu.hpp"

Mcu::Mcu(rclcpp::Node::SharedPtr node) : 
  logger_(node->get_logger()),
  serialService(), 
  serial(serialService)
{
  // ROS
  serialPortReconnectTimer = node->create_wall_timer(std::chrono::seconds(kWaitSecond), std::bind(&Mcu::AutoSearch, this));
  serialPortReconnectTimer->cancel();

  // Parameters
  auto mcuPortNameParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuPortNameParam.description = "Default serial port name or (Linux only) perform automated search if the this is unspecified.";
  node->declare_parameter("mcu_port_name", "", mcuPortNameParam);
  auto mcuResetTransformParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuResetTransformParam.description = "Reset robot transform on start up.";
  node->declare_parameter("mcu_reset_transform", false, mcuResetTransformParam);
  resetTransformOnConnected = node->get_parameter("mcu_reset_transform").as_bool();

  // Initalise
  std::string port = node->get_parameter("mcu_port_name").as_string();
  if(port.empty())
  {
    // Perform auto search if no port specified
    AutoSearch();
  }
  else
  {
    portName = port;
    Connect(portName);
  }
  GetSerialNumber();
}

Mcu::~Mcu()
{
  serialService.stop();
  if(ioThread.joinable())
    ioThread.join();
  serial.close();
}

void Mcu::StartSerialIo()
{
  if(ioThread.joinable()) 
  {
    serialService.restart();
    ioThread.join();
  }
  std::thread thread{[this](){ serialService.run(); }};
  ioThread.swap(thread);
}

void Mcu::AutoSearch()
{
  serialPortReconnectTimer->cancel();
  std::string port;
  std::filesystem::path path {"/dev"};
  for(auto const &file : std::filesystem::directory_iterator(path))
  {
    // Linux only, find first /dev/ttyACM*
    if(file.path().string().find("ttyACM") != std::string::npos)
    {
      port = file.path().string();
      RCLCPP_INFO(logger_, "Serial device %s found.", port.c_str());
      Connect(port);
      return;
    }  
  }
  RCLCPP_WARN(logger_, "No serial device found, try again in %d seconds.", kWaitSecond);
  serialPortReconnectTimer->reset();
}

void Mcu::Connect(const std::string &port)
{
  boost::system::error_code error;
  serial.open(port, error);
  if(error) 
  {
    RCLCPP_ERROR(logger_, "Serial connection throws an error: %s, try again in %d seconds.", error.message().c_str(), kWaitSecond);
    serialPortReconnectTimer->reset();
    return;
  }
  RCLCPP_INFO(logger_, "Serial port connected to %s", port.c_str());
  Read();
  if(resetTransformOnConnected)
  {
    resetTransformOnConnected = false;
    ResetTransformInternal();
  }
  StartSerialIo();
}

void Mcu::Reconnect()
{
  if(portName.empty()) // Perform auto search if no portName specified
    AutoSearch();
  else
    Connect(portName);
}

void Mcu::Read()
{
  serial.async_read_some(boost::asio::buffer(readBuffer, kReadBufferSize), std::bind(&Mcu::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2));    
}

void Mcu::OnReadComplete(boost::system::error_code error, std::size_t size)
{
  if(!serial.is_open())
    return; // Discard further error is the serial is closed

  if(!error)// OK
  {
    if(size >= 2) 
    {
      // Find the header and frame size
      if(readBuffer[0] == char(170)) // 170 = 0xAA
      {
        int frameSize = readBuffer[2];
        if(int(size) == frameSize)
        {
          // Process the data if received data = target size, 
          ProcessRobotData();
        }
      }
    }
    if(readBuffer[0] == (char) 171) // 171 = 0xAB
    {
      ProcessSerialNumber();
    }
    // Read more data
    Read();
  }
  else 
  {
    RCLCPP_ERROR(logger_, "Serial read throws an error: %s", error.message().c_str());
    //serialService.stop();
    serial.close();
    Reconnect();
  }
}

void Mcu::ProcessRobotData()
{
  int index = 3;
  robotData.refreshTime = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1]);
  index += 2;
  for(int i = 0; i < 3; i++)
  {
    uint32_t temp = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.transform[i] = Uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < 3; i++)
  {
    uint32_t temp = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.forwardKinematic[i] = Uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.targetWheelVelocity[i] = Uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.measuredWheelVelocity[i] = Uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.pConstant[i]  = Uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.iConstant[i]  = Uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < robotData.wheelCount; i++) 
  {
    uint32_t temp = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1], (uint8_t) readBuffer[index + 2], (uint8_t) readBuffer[index + 3]);
    robotData.dConstant[i]  = Uint32ToFloat(temp);
    index += 4;
  }
  for(int i = 0; i < 2; i++) 
  {
    robotData.battery[i] = CombineBytes((uint8_t) readBuffer[index], (uint8_t) readBuffer[index + 1]) * 0.004;
    index += 2;
  }
  uint8_t eStopByte = readBuffer[index];
  int compare = 128;
  for(int i = 0; i < 2; i++)
  {
    robotData.eStop[i] = (eStopByte & compare) >> (7 - i);
    compare = compare >> 1;
  }
  //updateDeamon(robotData);
}

void Mcu::CharArrayToHex(const char* input, size_t length, char* output) 
{
  const char hexChars[] = "0123456789ABCDEF";
  for (size_t i = 0; i < length; ++i) 
  {
    output[i * 2] = hexChars[(input[i] >> 4) & 0xF];  
    output[i * 2 + 1] = hexChars[input[i] & 0xF]; 
  }
  output[length * 2] = '\0';
}

void Mcu::GetSerialNumber()
{
  std::vector<char> ba(1);
  ba[0] = 'S';
  Write(ba);
}

void Mcu::ProcessSerialNumber()
{
  char sn[30] = {0};
  CharArrayToHex(&readBuffer[1], 12, sn);
  //serialNumber(sn);
}

void Mcu::ResetTransformInternal()
{
  std::vector<char> ba(1);
  ba[0] = 'T';
  Write(ba);
}

void Mcu::Write(const std::vector<char> &data)
{
  if(serial.is_open())
    serial.async_write_some(boost::asio::buffer(data), std::bind(&Mcu::OnWriteComplete, this, std::placeholders::_1));
}

void Mcu::OnWriteComplete(boost::system::error_code error)
{ 
  if(error) 
  {
    RCLCPP_ERROR(logger_, "Serial read throws an error: %s", error.message().c_str());
  }
}

void Mcu::SetInverseKinematics(float x, float y, float w)
{
  uint32_t ux = FloatToUint32(x);
  uint32_t uy = FloatToUint32(y);
  uint32_t uw = FloatToUint32(w);
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
  Write(ba);
}

void Mcu::SetEstop(int enable)
{
  std::vector<char> ba(5);
  ba[0] = 'E';
  ba[1] = (enable & 4278190080) >> 24;
  ba[2] = (enable & 16711680) >> 16;
  ba[3] = (enable & 65280) >> 8;
  ba[4] = enable & 255;
  Write(ba);
}

void Mcu::ResetTransform()
{
  if(serial.is_open())
    ResetTransformInternal();
  else
    resetTransformOnConnected = true;
}

void Mcu::SetExternalImu(float ax, float ay, float az, float gz)
{
  uint32_t ux = FloatToUint32(ax);
  uint32_t uy = FloatToUint32(ay);
  uint32_t uz = FloatToUint32(az);
  uint32_t ugz = FloatToUint32(gz);
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
  Write(ba);
}
