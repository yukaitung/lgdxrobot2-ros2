#ifndef CLOUDADAPTER_HPP
#define CLOUDADAPTER_HPP

#include <memory>
#include <string>
#include <functional>

#include "proto/RobotClientsService.grpc.pb.h"
#include "RealtimeExchange.hpp"

enum class CloudFunctions
{
  Greet = 0,
  Exchange,
  AutoTaskNext,
  AutoTaskAbort
};

class CloudAdapter
{
  private:
    const int kGrpcWaitSec = 5;
    std::shared_ptr<grpc::Channel> grpcChannel;
    std::unique_ptr<RobotClientsService::Stub> grpcStub;
    std::unique_ptr<RobotClientsService::Stub> grpcRealtimeStub;
    std::unique_ptr<RealtimeExchange> realtimeExchange;
    std::function<void(void)> startNextExchange;
    std::function<void(const RobotClientsRespond *)> updateDeamon;
    std::function<void(const char *, int)> log;
    std::function<void(CloudFunctions)> error;
    std::function<void(bool)> setRealtimeExchange;
    std::shared_ptr<grpc::CallCredentials> accessToken;
    RobotClientsRobotCommands robotCommand;

    std::string readCert(const char *filename);
    #ifdef __linux__ 
    std::string getMotherBoardSerialNumber();
    #endif
    void setSystemInfo(RobotClientsSystemInfo *info);

  public:
    CloudAdapter(const char *serverAddress,
      const char *rootCertPath,
      const char *clientKeyPath,
      const char *clientCertPath,
      std::function<void(void)> startNextExchangeCb,
      std::function<void(const RobotClientsRespond *)> updateDaemonCb,
      std::function<void(const char *, int)> logCb,
      std::function<void(CloudFunctions)> errorCb,
      std::function<void(bool)> setRealtimeExchangeCb);
    void greet(std::string mcuSerialNumber);
    void exchange(RobotClientsRobotStatus robotStatus,
      RobotClientsRobotCriticalStatus &criticalStatus,
      std::vector<double> &batteries,
      RobotClientsDof &position,
      RobotClientsAutoTaskNavProgress &navProgress);
    void exchangeStream(RobotClientsRobotStatus robotStatus,
      RobotClientsRobotCriticalStatus &criticalStatus,
      std::vector<double> &batteries,
      RobotClientsDof &position,
      RobotClientsAutoTaskNavProgress &navProgress);
    void autoTaskNext(RobotClientsNextToken &token);
    void autoTaskAbort(RobotClientsAbortToken &token);
    void shutdown();
};

#endif // CLOUDADAPTER_HPP