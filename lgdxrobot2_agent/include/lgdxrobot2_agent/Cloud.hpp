#ifndef CLOUD_HPP
#define CLOUD_HPP

#include <queue>

#include "CloudExchangeStream.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "RobotStatus.hpp"
#include "Structs/CloudSignals.hpp"

#include "lgdxrobot2_agent/msg/auto_task.hpp"
#include "lgdxrobot2_agent/srv/auto_task_abort.hpp"
#include "lgdxrobot2_agent/srv/auto_task_next.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

enum class CloudFunctions
{
  Greet = 0,
  Exchange,
  AutoTaskNext,
  AutoTaskAbort
};

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
    std::unique_ptr<RobotClientsService::Stub> grpcRealtimeStub;
    std::unique_ptr<RobotClientsService::Stub> grpcStub;

    bool isRealtimeExchange = false;
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

    void Error(CloudFunctions function);
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
    void Shutdown();
};

#endif // CLOUD_HPP