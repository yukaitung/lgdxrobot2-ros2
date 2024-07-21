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
    std::function<void(const RpcRespond *)> respondCallback;
    std::function<void(const char *, int)> debugCallback;

    std::string readCert(const char *filename);

  public:
    CloudAdapter(const char *serverAddress,
      const char *rootCertPath,
      const char *clientCertPath,
      const char *clientKeyPath,
      std::function<void(const RpcRespond *)> respondCb,
      std::function<void(const char *, int)> debugCb);
    void greet(RpcGreet &greet);
    void exchange(RpcExchange &exchange);
    void autoTaskNext(RpcCompleteToken &token);
    void autoTaskAbort(RpcCompleteToken &token);
};

#endif // CLOUDADAPTER_HPP