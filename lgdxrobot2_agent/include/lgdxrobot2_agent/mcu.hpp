#ifndef MCU_HPP
#define MCU_HPP

#include <array>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include "rclcpp/rclcpp.hpp"

#include "lgdxrobot2.h"
#include "mcu_signals.hpp"

class Mcu
{
  public:
    Mcu(rclcpp::Node::SharedPtr node, std::shared_ptr<McuSignals> mcu_signals_ptr);
    ~Mcu();

    void SetInverseKinematics(float x, float y, float w);
    void SetEstop(int enable);
    void ResetTransform();

  private:
    // Constants
    const int kWaitSecond = 3;

    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr serial_port_reconnect_timer_;

    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_;
    std::thread io_thread_;

    std::shared_ptr<McuSignals> mcu_signals_;

    // Settings
    std::string serial_port_name_;
    bool reset_transform_on_connected_ = false;

    // Data
    McuData mcu_data_;
    std::array<uint8_t, 512> read_buffer_ = {0};
    boost::circular_buffer<uint8_t> mcu_buffer_;

    // io_context thread
    void StartSerialIo();

    // Connection
    void Connect();

    // Read from MCU
    boost::asio::awaitable<void> Read();
    void OnReadComplete(size_t size);

    // Write to MCU
    void ResetTransformInternal();
    boost::asio::awaitable<void> Write(const std::vector<char> data);
};

#endif // MCU_HPP