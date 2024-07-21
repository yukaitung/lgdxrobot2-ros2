#ifndef CLOUDADAPTER_HPP
#define CLOUDADAPTER_HPP

#include <memory>
#include <string>

#include "proto/RobotClientService.grpc.pb.h"

class CloudAdapter
{
  private:
    std::shared_ptr<grpc::Channel> grpcChannel;
    std::unique_ptr<RobotClientService::Stub> grpcStub;

    std::string readCert(const char *filename);

  public:
    CloudAdapter(const char *serverAddress,
      const char *rootCertPath,
      const char *clientCertPath,
      const char *clientKeyPath);
};

#endif // CLOUDADAPTER_HPP