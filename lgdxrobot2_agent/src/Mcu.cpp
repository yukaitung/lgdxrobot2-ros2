#include <filesystem>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <boost/asio/co_spawn.hpp>

#include "lgdxrobot2_agent/Mcu.hpp"

Mcu::Mcu(rclcpp::Node::SharedPtr node, std::shared_ptr<McuSignals> mcuSignalsPtr) :
  _node(node),
  _logger(node->get_logger()),
  ioContext(), 
  serial(ioContext)
{
  mcuSignals = mcuSignalsPtr;

  // ROS
  serialPortReconnectTimer = node->create_wall_timer(std::chrono::seconds(kWaitSecond), std::bind(&Mcu::Connect, this));
  serialPortReconnectTimer->cancel();

  // Parameters
  resetTransformOnConnected = node->get_parameter("reset_transform").as_bool();

  // Initalise
  Connect();
}

void Mcu::Shutdown()
{
  ioContext.stop();
  if(ioThread.joinable())
    ioThread.join();
  serial.close();
}

void Mcu::StartSerialIo()
{
  if(ioThread.joinable()) 
  {
    ioContext.restart();
    ioThread.join();
  }
  std::thread thread{[this](){ ioContext.run(); }};
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
  
  boost::asio::co_spawn(ioContext, Mcu::Read(), boost::asio::detached);

  if(resetTransformOnConnected)
  {
    resetTransformOnConnected = false;
    ResetTransformInternal();
  }

  StartSerialIo();
}

awaitable<void> Mcu::Read()
{
  try
  {
    while (rclcpp::ok())
    {
      if (!serial.is_open())
        break;

      std::size_t size = co_await serial.async_read_some(boost::asio::buffer(readBuffer), boost::asio::use_awaitable); 
      OnReadComplete(size);
    }
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(_logger, "Serial read throws an error: %s", e.what());
    serial.close();
    serialPortReconnectTimer->reset();
  } 
}

void Mcu::OnReadComplete(size_t size)
{
  const uint8_t startSeq[] = {0xAA, 0x55};
  const uint8_t endSeq[] = {0xA5, 0x5A};

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

  std::vector<uint8_t> frame(startIt, nextIt + sizeof(endSeq));
  if (frame.size() > 3)
  {
    switch (frame.at(2))
    {
      case MCU_DATA_TYPE:
        memcpy(&mcuData, frame.data(), sizeof(McuData));
        mcuSignals->UpdateMcuData(mcuData);
        break;
      default:
        break;
    }
  }

  // Remove processed data from buffer
  mcuBuffer.erase(startIt, nextIt + sizeof(endSeq));
}

void Mcu::ResetTransformInternal()
{
  McuResetTransformCommand command;
  command.header1 = MCU_HEADER1;
  command.header2 = MCU_HEADER2;
  command.command = MCU_RESET_TRANSFORM_COMMAND_TYPE;
  std::vector<char> buffer(sizeof(McuResetTransformCommand));
  std::memcpy(buffer.data(), &command, sizeof(McuResetTransformCommand));
  boost::asio::co_spawn(ioContext, Mcu::Write(buffer), boost::asio::detached);
}

awaitable<void> Mcu::Write(const std::vector<char> data)
{
  if(serial.is_open())
  {
    try
    {
      co_await serial.async_write_some(boost::asio::buffer(data), boost::asio::use_awaitable);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(_logger, "Serial write throws an error: %s", e.what());
    }
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
  boost::asio::co_spawn(ioContext, Mcu::Write(buffer), boost::asio::detached);
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
  boost::asio::co_spawn(ioContext, Mcu::Write(buffer), boost::asio::detached);
}

void Mcu::ResetTransform()
{
  if(serial.is_open())
    ResetTransformInternal();
  else
    resetTransformOnConnected = true;
}
