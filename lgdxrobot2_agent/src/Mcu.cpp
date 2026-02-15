#include <filesystem>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "lgdxrobot2_agent/Mcu.hpp"

Mcu::Mcu(rclcpp::Node::SharedPtr node, std::shared_ptr<McuSignals> mcuSignalsPtr) :
  _node(node),
  _logger(node->get_logger()),
  serialService(), 
  serial(serialService)
{
  mcuSignals = mcuSignalsPtr;

  // ROS
  serialPortReconnectTimer = node->create_wall_timer(std::chrono::seconds(kWaitSecond), std::bind(&Mcu::Connect, this));
  serialPortReconnectTimer->cancel();

  // Parameters
  auto mcuPortNameParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuPortNameParam.description = "Serial port name for the LGDXRobot2 or default to /dev/lgdxrobot2.";
  node->declare_parameter("serial_port_name", "/dev/lgdxrobot2", mcuPortNameParam);
  auto mcuResetTransformParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuResetTransformParam.description = "Reset robot transform on start up.";
  node->declare_parameter("reset_transform", false, mcuResetTransformParam);
  resetTransformOnConnected = node->get_parameter("reset_transform").as_bool();

  // Initalise
  Connect();
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

void Mcu::Connect()
{
  std::string port = _node->get_parameter("serial_port_name").as_string();
  RCLCPP_INFO(_logger, "Attempting to connect to %s", port.c_str());

  boost::system::error_code error;
  serial.open(port, error);
  if(error) 
  {
    RCLCPP_ERROR(_logger, "Serial connection throws an error: %s, try again in %d seconds.", error.message().c_str(), kWaitSecond);
    serialPortReconnectTimer->reset();
    return;
  }
  RCLCPP_INFO(_logger, "Serial port connected to %s", port.c_str());
  serialPortReconnectTimer->cancel();
  Read();
  if(resetTransformOnConnected)
  {
    resetTransformOnConnected = false;
    ResetTransformInternal();
  }
  StartSerialIo();
}

void Mcu::Read()
{
  serial.async_read_some(boost::asio::buffer(readBuffer), std::bind(&Mcu::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2));    
}

void Mcu::OnReadComplete(boost::system::error_code error, std::size_t size)
{
  const uint8_t startSeq[] = {0xAA, 0x55};
  const uint8_t endSeq[] = {0xA5, 0x5A};

  if(!serial.is_open())
    return; // Discard further error is the serial is closed

  if(!error) // OK
  {
    mcuBuffer.insert(mcuBuffer.end(), readBuffer.begin(), readBuffer.begin() + size);

    auto startIt = std::search(mcuBuffer.begin(), mcuBuffer.end(), std::begin(startSeq), std::end(startSeq));
    if (startIt == mcuBuffer.end()) 
    {
      // No frame start found, discard junk
      mcuBuffer.clear();
      return;
    }

    auto nextIt = std::search(startIt + 2, mcuBuffer.end(), std::begin(endSeq), std::end(endSeq));
    if (nextIt == mcuBuffer.end()) {
      // No frame end found yet
      return;
    }

    auto lastStartIt = startIt;
    bool mcuDataFound = false;
    // Handle frames unitl no complete frame is found
    while (startIt != mcuBuffer.end() && nextIt != mcuBuffer.end())
    {
      std::vector<uint8_t> frame(startIt, nextIt + sizeof(endSeq));
      if (frame.size() > 3)
      {
        switch (frame.at(2))
        {
          case MCU_DATA_TYPE:
            if (!mcuDataFound)
            {
              memcpy(&mcuData, frame.data(), sizeof(McuData));
              mcuSignals->UpdateMcuData(mcuData);
              mcuDataFound = true;
            }
            break;
          case MCU_SERIAL_NUMBER_TYPE:
            McuSerialNumber mcuSerialNumber;
            memcpy(&mcuSerialNumber, frame.data(), sizeof(McuSerialNumber));
            ProcessSerialNumber(mcuSerialNumber);
          default:
            break;
        }
      }

      lastStartIt = startIt;
      // Find next frame start
      startIt = std::search(nextIt + 2, mcuBuffer.end(), std::begin(startSeq), std::end(startSeq));
      nextIt = std::search(startIt + 2, mcuBuffer.end(), std::begin(endSeq), std::end(endSeq));
    }

    // Remove processed data from buffer
	  mcuBuffer.erase(mcuBuffer.begin(), lastStartIt);

    // Read more data
    Read();
  }
  else 
  {
    RCLCPP_ERROR(_logger, "Serial read throws an error: %s", error.message().c_str());
    //serialService.stop();
    serial.close();
    Connect();
  }
}

std::string Mcu::SerialToHexString(uint32_t serial1, uint32_t serial2, uint32_t serial3) 
{
  std::ostringstream oss;

  // Convert each uint32_t to 8-character hex with leading zeros
  oss << std::hex << std::uppercase << std::setfill('0')
      << std::setw(8) << serial1
      << std::setw(8) << serial2
      << std::setw(8) << serial3;

  return oss.str();
}

void Mcu::GetSerialNumber()
{
  McuGetSerialNumberCommand command;
  command.header1 = MCU_HEADER1;
  command.header2 = MCU_HEADER2;
  command.command = MCU_GET_SERIAL_NUMBER_COMMAND_TYPE;
  std::vector<char> buffer(sizeof(McuGetSerialNumberCommand));
  std::memcpy(buffer.data(), &command, sizeof(McuGetSerialNumberCommand));
  Write(buffer);
}

void Mcu::ProcessSerialNumber(const McuSerialNumber &mcuSerialNumber)
{
  if (hasSerialNumber)
  {
    return;
  }
  hasSerialNumber = true;
  std::string sn = SerialToHexString(mcuSerialNumber.serial_number1, mcuSerialNumber.serial_number2, mcuSerialNumber.serial_number3);
  RCLCPP_INFO(_logger, "Serial Number received: %s", sn.c_str());
  mcuSignals->UpdateSerialNumber(sn);
}

void Mcu::ResetTransformInternal()
{
  McuResetTransformCommand command;
  command.header1 = MCU_HEADER1;
  command.header2 = MCU_HEADER2;
  command.command = MCU_RESET_TRANSFORM_COMMAND_TYPE;
  std::vector<char> buffer(sizeof(McuResetTransformCommand));
  std::memcpy(buffer.data(), &command, sizeof(McuResetTransformCommand));
  Write(buffer);
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
    RCLCPP_ERROR(_logger, "Serial write throws an error: %s", error.message().c_str());
  }
}

void Mcu::SetInverseKinematics(float x, float y, float w)
{
  McuInverseKinematicsCommand command;
  command.header1 = MCU_HEADER1;
  command.header2 = MCU_HEADER2;
  command.command = MCU_INVERSE_KINEMATICS_COMMAND_TYPE;
  command.velocity.x = x;
  command.velocity.y = y;
  command.velocity.rotation = w;
  std::vector<char> buffer(sizeof(McuInverseKinematicsCommand));
  std::memcpy(buffer.data(), &command, sizeof(McuInverseKinematicsCommand));
  Write(buffer);
}

void Mcu::SetEstop(int enable)
{
  McuSoftwareEmergencyStopCommand command;
  command.header1 = MCU_HEADER1;
  command.header2 = MCU_HEADER2;
  command.command = MCU_SOFTWARE_EMERGENCY_STOP_COMMAND_TYPE;
  command.enable = enable;
  std::vector<char> buffer(sizeof(McuSoftwareEmergencyStopCommand));
  std::memcpy(buffer.data(), &command, sizeof(McuSoftwareEmergencyStopCommand));
  Write(buffer);
}

void Mcu::ResetTransform()
{
  if(serial.is_open())
    ResetTransformInternal();
  else
    resetTransformOnConnected = true;
}
