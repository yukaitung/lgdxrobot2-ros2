#ifndef CLOUD_HPP
#define CLOUD_HPP

#include <queue>

#include "CloudExchangeStream.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "RobotStatus.hpp"
#include "SlamExchangeStream.hpp"
#include "Structs/CloudSignals.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

struct CloudErrorRetryData
{
  std::string mcuSerialNumber;
  RobotClientsNextToken nextToken;
  RobotClientsAbortToken abortToken;
};

class Cloud
{
  private:
    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr cloudRetryTimer;

    const int kGrpcWaitSec = 5;

    std::shared_ptr<grpc::Channel> grpcChannel;
    std::unique_ptr<CloudExchangeStream> cloudExchangeStream;
    std::unique_ptr<SlamExchangeStream> slamExchangeStream;
    std::unique_ptr<RobotClientsService::Stub> grpcRealtimeStub;
    std::unique_ptr<RobotClientsService::Stub> grpcStub;

    bool isCloudSlam = false;
    bool hasError = false;
    CloudErrorRetryData cloudErrorRetryData;
    std::shared_ptr<grpc::CallCredentials> accessToken;
    RobotClientsRobotCommands robotCommand;
    std::shared_ptr<CloudSignals> cloudSignals;

    std::string ReadCertificate(const char *filename);
    #ifdef __linux__ 
    std::string GetMotherBoardSerialNumber();
    #endif
    void SetSystemInfo(RobotClientsSystemInfo *info);
    void HandleError();

  public:
    Cloud(rclcpp::Node::SharedPtr node,
      std::shared_ptr<CloudSignals> cloudSignalsPtr);
    void Greet(std::string mcuSN);
    void Exchange(const RobotClientsData &robotData,
      const RobotClientsNextToken &nextToken,
      const RobotClientsAbortToken &abortToken);
    void SlamExchange(const RobotClientsSlamStatus status,
      const RobotClientsData &robotData,
      const RobotClientsMapData &mapData);
    void OnErrorOccured();
    void Shutdown();
};

#endif // CLOUD_HPP