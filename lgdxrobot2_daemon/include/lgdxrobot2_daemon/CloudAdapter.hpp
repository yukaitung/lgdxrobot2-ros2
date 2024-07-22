#ifndef CLOUDADAPTER_HPP
#define CLOUDADAPTER_HPP

#include <memory>
#include <string>
#include <functional>

#include "proto/RobotClientService.grpc.pb.h"

class CloudAdapter
{
  private:
    std::shared_ptr<grpc::Channel> grpcChannel;
    std::unique_ptr<RobotClientService::Stub> grpcStub;
    std::function<void(const RpcRespond *)> updateDeamon;
    std::function<void(const char *, int)> log;

    RpcRobotSystemInfo systemInfo;
    RpcGreet greet;

    std::string readCert(const char *filename);
    RpcRobotSystemInfo GenerateSystemInfo();
    RpcGreet GenerateGreet(RpcRobotSystemInfo *systemInfo);


  public:
    CloudAdapter(const char *serverAddress,
      const char *rootCertPath,
      const char *clientCertPath,
      const char *clientKeyPath,
      std::function<void(const RpcRespond *)> updateDaemonCb,
      std::function<void(const char *, int)> logCb);
    void grpcGreet(RpcGreet &greet);
    void exchange(RpcExchange &exchange);
    void autoTaskNext(RpcCompleteToken &token);
    void autoTaskAbort(RpcCompleteToken &token);
};

#endif // CLOUDADAPTER_HPP