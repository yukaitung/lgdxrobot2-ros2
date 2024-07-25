#ifndef CLOUDADAPTER_HPP
#define CLOUDADAPTER_HPP

#include <memory>
#include <string>
#include <functional>

#include "proto/RobotClientService.grpc.pb.h"

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
    std::unique_ptr<RobotClientService::Stub> grpcStub;
    std::function<void(const RpcRespond *)> updateDeamon;
    std::function<void(const char *, int)> log;
    std::function<void(CloudFunctions)> error;

    std::string readCert(const char *filename);
    void setSystemInfo(RpcRobotSystemInfo *info);

  public:
    CloudAdapter(const char *serverAddress,
      const char *rootCertPath,
      const char *clientCertPath,
      const char *clientKeyPath,
      std::function<void(const RpcRespond *)> updateDaemonCb,
      std::function<void(const char *, int)> logCb,
      std::function<void(CloudFunctions)> errorCb);
    void greet();
    void exchange(RpcExchange &exchange);
    void autoTaskNext(RpcCompleteToken &token);
    void autoTaskAbort(RpcCompleteToken &token);
};

#endif // CLOUDADAPTER_HPP