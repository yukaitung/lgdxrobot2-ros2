#include <filesystem>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "lgdxrobot2_agent/Mcu.hpp"

Mcu::Mcu(rclcpp::Node::SharedPtr node, std::shared_ptr<McuSignals> mcuSignalsPtr) :
  logger_(node->get_logger()),
  serialService(), 
  serial(serialService)
{
  mcuSignals = mcuSignalsPtr;

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
              McuData mcuData;
              memcpy(&mcuData, frame.data(), sizeof(McuData));
              ProcessRobotData(mcuData);
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
    RCLCPP_ERROR(logger_, "Serial read throws an error: %s", error.message().c_str());
    //serialService.stop();
    serial.close();
    Reconnect();
  }
}

void Mcu::ProcessRobotData(const McuData &mcuData)
{
  robotData.responseTime = mcuData.response_time;
  robotData.transform[0] = mcuData.transform.x;
  robotData.transform[1] = mcuData.transform.y;
  robotData.transform[2] = mcuData.transform.rotation;
  robotData.forwardKinematic[0] = mcuData.forward_kinematic.x;
  robotData.forwardKinematic[1] = mcuData.forward_kinematic.y;
  robotData.forwardKinematic[2] = mcuData.forward_kinematic.rotation;
  std::copy(mcuData.motors_target_velocity, mcuData.motors_target_velocity + 4, robotData.motorsTargetVelocity);
  std::copy(mcuData.motors_desire_velocity, mcuData.motors_desire_velocity + 4, robotData.motorsDesireVelocity);
  std::copy(mcuData.motors_actual_velocity, mcuData.motors_actual_velocity + 4, robotData.motorsActualVelocity);
  std::copy(mcuData.motors_ccr, mcuData.motors_ccr + 4, robotData.motorsCcr);
  robotData.batteryCurrent[0] = mcuData.battery1.current;
  robotData.batteryCurrent[1] = mcuData.battery2.current;
  robotData.batteryVoltage[0] = mcuData.battery1.voltage;
  robotData.batteryVoltage[1] = mcuData.battery2.voltage;
  robotData.eStop[0] = mcuData.software_emergency_stop_enabled;
  robotData.eStop[1] = mcuData.hardware_emergency_stop_enabled;
  robotData.eStop[2] = mcuData.bettery_low_emergency_stop_enabled;
  mcuSignals->UpdateRobotData(robotData);
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
  command.command = MCU_GET_SERIAL_NUMBER_COMMAND_TYPE;
  std::vector<char> buffer(sizeof(McuGetSerialNumberCommand));
  std::memcpy(buffer.data(), &command, sizeof(McuGetSerialNumberCommand));
  Write(buffer);
}

void Mcu::ProcessSerialNumber(const McuSerialNumber &mcuSerialNumber)
{
  std::string sn = SerialToHexString(mcuSerialNumber.serial_number1, mcuSerialNumber.serial_number2, mcuSerialNumber.serial_number3);
  mcuSignals->UpdateSerialNumber(sn);
}

void Mcu::ResetTransformInternal()
{
  McuResetTransformCommand command;
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
    RCLCPP_ERROR(logger_, "Serial read throws an error: %s", error.message().c_str());
  }
}

void Mcu::SetInverseKinematics(float x, float y, float w)
{
  McuInverseKinematicsCommand command;
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
