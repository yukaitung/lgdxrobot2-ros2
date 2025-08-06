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

    bool isRealtimeExchange = false;
    bool isCloudSlam = false;
    std::queue<CloudFunctions> cloudErrorQueue;
    CloudErrorRetryData cloudErrorRetryData;
    std::shared_ptr<grpc::CallCredentials> accessToken;
    RobotClientsRobotCommands robotCommand;
    std::shared_ptr<RobotStatus> robotStatus;
    std::shared_ptr<CloudSignals> cloudSignals;

    std::string ReadCertificate(const char *filename);
    #ifdef __linux__ 
    std::string GetMotherBoardSerialNumber();
    #endif
    void SetSystemInfo(RobotClientsSystemInfo *info);

    void ExchangePolling(RobotClientsRobotCriticalStatus &criticalStatus,
      std::vector<double> &batteries,
      RobotClientsDof &position,
      RobotClientsAutoTaskNavProgress &navProgress);
    void ExchangeStream(RobotClientsRobotCriticalStatus &criticalStatus,
      std::vector<double> &batteries,
      RobotClientsDof &position,
      RobotClientsAutoTaskNavProgress &navProgress);
    void HandleError();

  public:
    Cloud(rclcpp::Node::SharedPtr node,
      std::shared_ptr<CloudSignals> cloudSignalsPtr,
      std::shared_ptr<RobotStatus> robotStatusPtr);
    void Greet(std::string mcuSN);
    void Exchange(RobotClientsRobotCriticalStatus &criticalStatus,
      std::vector<double> &batteries,
      RobotClientsDof &position,
      RobotClientsAutoTaskNavProgress &navProgress);
    void AutoTaskNext(RobotClientsNextToken &token);
    void AutoTaskAbort(RobotClientsAbortToken &token);
    void SlamExchange(RobotClientsSlamStatus status,
      RobotClientsExchange &exchange);
    void SlamExchange(RobotClientsSlamStatus status,
      RobotClientsExchange &exchange,
      RobotClientsMapData &mapData);
    void Shutdown();
    void Error(CloudFunctions function);
};

#endif // CLOUD_HPP