#include <algorithm>
#include <bit>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <boost/asio/co_spawn.hpp>

#include "lgdxrobot2_agent/mcu.hpp"

Mcu::Mcu(rclcpp::Node::SharedPtr node, std::shared_ptr<McuSignals> mcu_signals_ptr) :
  logger_(node->get_logger()),
  io_context_(), 
  serial_(io_context_),
  mcu_buffer_(512)
{
  mcu_signals_ = mcu_signals_ptr;
  serial_port_name_ =  node->get_parameter("serial_port_name").as_string();

  // ROS
  serial_port_reconnect_timer_ = node->create_wall_timer(std::chrono::seconds(kWaitSecond), std::bind(&Mcu::Connect, this));
  serial_port_reconnect_timer_->cancel();

  // Parameters
  reset_transform_on_connected_ = node->get_parameter("reset_transform").as_bool();

  // Initalise
  Connect();
}

Mcu::~Mcu()
{
  if (serial_.is_open())
  {
    serial_.cancel();
    serial_.close();
  }
  io_context_.stop();
  if(io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void Mcu::StartSerialIo()
{
  if(io_thread_.joinable()) 
  {
    io_context_.restart();
    io_thread_.join();
  }
  std::thread thread{[this](){ io_context_.run(); }};
  io_thread_.swap(thread);
}

void Mcu::Connect()
{
  RCLCPP_INFO(logger_, "Attempting to connect to %s", serial_port_name_.c_str());

  boost::system::error_code error;
  serial_.open(serial_port_name_, error);
  if(error) 
  {
    RCLCPP_ERROR(logger_, "Serial connection throws an error: %s, try again in %d seconds.", error.message().c_str(), kWaitSecond);
    serial_port_reconnect_timer_->reset();
    return;
  }
  RCLCPP_INFO(logger_, "Serial port connected to %s", serial_port_name_.c_str());
  serial_port_reconnect_timer_->cancel();
  
  boost::asio::co_spawn(io_context_, Mcu::Read(), boost::asio::detached);

  if(reset_transform_on_connected_)
  {
    reset_transform_on_connected_ = false;
    ResetTransformInternal();
  }

  StartSerialIo();
}

boost::asio::awaitable<void> Mcu::Read()
{
  try
  {
    while (rclcpp::ok())
    {
      if (!serial_.is_open())
      {
        break;
      }
        
      std::size_t size = co_await serial_.async_read_some(boost::asio::buffer(read_buffer_), boost::asio::use_awaitable); 
      OnReadComplete(size);
    }
  }
  catch(const std::exception& e)
  {
    if (rclcpp::ok())
    {
      RCLCPP_ERROR(logger_, "Serial read error encountered, reconnection required: %s", e.what());
      serial_.cancel();
      serial_.close();
      serial_port_reconnect_timer_->reset();
    }
  } 
}

void Mcu::OnReadComplete(size_t size)
{
  const uint8_t start_seq[] = {0xAA, 0x55};
  const uint8_t end_seq[] = {0xA5, 0x5A};

  mcu_buffer_.insert(mcu_buffer_.end(), read_buffer_.begin(), read_buffer_.begin() + size);

  auto start_it = std::search(mcu_buffer_.begin(), mcu_buffer_.end(), std::begin(start_seq), std::end(start_seq));
  if (start_it == mcu_buffer_.end()) 
  {
    // No frame start found, discard junk
    mcu_buffer_.clear();
    return;
  }

  auto next_it = std::search(start_it + 2, mcu_buffer_.end(), std::begin(end_seq), std::end(end_seq));
  if (next_it == mcu_buffer_.end()) {
    // No frame end found yet
    return;
  }

  std::array<uint8_t, sizeof(McuData)> frame;
  std::copy(start_it, next_it + sizeof(end_seq), frame.begin());
  if (frame.size() > 3)
  {
    switch (frame.at(2))
    {
      case MCU_DATA_TYPE:
        mcu_data_ = std::bit_cast<McuData>(frame);
        mcu_signals_->update_mcu_data(mcu_data_);
        break;
      default:
        break;
    }
  }

  // Remove processed data from buffer
  mcu_buffer_.erase(start_it, next_it + sizeof(end_seq));
}

void Mcu::ResetTransformInternal()
{
  McuResetTransformCommand command;
  command.header1 = MCU_HEADER1;
  command.header2 = MCU_HEADER2;
  command.command = MCU_RESET_TRANSFORM_COMMAND_TYPE;
  std::vector<char> buffer(sizeof(McuResetTransformCommand));
  std::memcpy(buffer.data(), &command, sizeof(McuResetTransformCommand));
  boost::asio::co_spawn(io_context_, Mcu::Write(std::move(buffer)), boost::asio::detached);
}

boost::asio::awaitable<void> Mcu::Write(const std::vector<char> data)
{
  if(serial_.is_open())
  {
    try
    {
      co_await serial_.async_write_some(boost::asio::buffer(data), boost::asio::use_awaitable);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "Serial write throws an error: %s", e.what());
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
  boost::asio::co_spawn(io_context_, Mcu::Write(std::move(buffer)), boost::asio::detached);
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
  boost::asio::co_spawn(io_context_, Mcu::Write(std::move(buffer)), boost::asio::detached);
}

void Mcu::ResetTransform()
{
  if(serial_.is_open())
    ResetTransformInternal();
  else
    reset_transform_on_connected_ = true;
}
