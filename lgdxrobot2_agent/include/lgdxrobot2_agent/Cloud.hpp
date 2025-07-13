#ifndef CLOUD_HPP
#define CLOUD_HPP

#include "CloudExchangeStream.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "RobotStatus.hpp"

#include "lgdxrobot2_agent/msg/auto_task.hpp"
#include "lgdxrobot2_agent/srv/auto_task_abort.hpp"
#include "lgdxrobot2_agent/srv/auto_task_next.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

enum class CloudFunctions
{
  Greet = 0,
  Exchange,
  AutoTaskNext,
  AutoTaskAbort
};

class Cloud
{
  private:
    rclcpp::Logger logger_;

    rclcpp::TimerBase::SharedPtr cloudRetryTimer;
    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr crtitcalStatusPublisher;
    rclcpp::Publisher<lgdxrobot2_agent::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;

    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    const int kGrpcWaitSec = 5;
    std::shared_ptr<grpc::Channel> grpcChannel;
    std::unique_ptr<RobotClientsService::Stub> grpcStub;
    std::unique_ptr<RobotClientsService::Stub> grpcRealtimeStub;
    std::shared_ptr<grpc::CallCredentials> accessToken;
    std::unique_ptr<CloudExchangeStream> cloudExchangeStream;
    RobotClientsRobotCommands robotCommand;
    std::shared_ptr<RobotStatus> robotStatus;

    std::string ReadCertificate(const char *filename);
    #ifdef __linux__ 
    std::string GetMotherBoardSerialNumber();
    #endif
    void SetSystemInfo(RobotClientsSystemInfo *info);

  public:
    Cloud(rclcpp::Node::SharedPtr node,
      std::shared_ptr<RobotStatus> robotStatusPtr);
    void Greet();
    void Exchange();
    void ExchangeStream();
    void AutoTaskNext(RobotClientsNextToken &token);
    void AutoTaskAbort(RobotClientsAbortToken &token);
    void Shutdown();
};

#endif // CLOUD_HPP