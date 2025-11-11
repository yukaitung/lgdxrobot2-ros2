#ifndef SLAMEXCHANGESTREAM_HPP
#define SLAMEXCHANGESTREAM_HPP

#include <grpcpp/grpcpp.h>
#include <mutex>
#include <condition_variable>

#include "Structs/CloudSignals.hpp"
#include "proto/RobotClientsService.grpc.pb.h"

class SlamExchangeStream : public grpc::ClientBidiReactor<RobotClientsSlamExchange, RobotClientsSlamCommands>
{
  private:
    std::shared_ptr<CloudSignals> cloudSignals;
    grpc::ClientContext context;
    RobotClientsSlamExchange *requestPtr;
    RobotClientsSlamCommands respond;
    std::mutex mutex;
    std::condition_variable cv;
    grpc::Status finalStatus;
    bool isShutdown = false;
    bool done = false;

    void OnWriteDone(bool ok) override;
    void OnReadDone(bool ok) override;
    void OnDone(const grpc::Status& status) override;

  public:
    SlamExchangeStream(RobotClientsService::Stub *stub, 
      std::shared_ptr<grpc::CallCredentials> accessToken,
      std::shared_ptr<CloudSignals> cloudSignalsPtr);
    ~SlamExchangeStream();
    void SendMessage(const RobotClientsSlamStatus status,
      const RobotClientsData &robotData,
      const RobotClientsMapData &mapData);
    void Shutdown();
    grpc::Status AwaitCompletion();
};

#endif // SLAMEXCHANGESTREAM_HPP