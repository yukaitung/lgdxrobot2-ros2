#ifndef CLOUDEXCHANGESTREAM_HPP
#define CLOUDEXCHANGESTREAM_HPP

#include <grpcpp/grpcpp.h>
#include <mutex>
#include <condition_variable>

#include "Structs/CloudSignals.hpp"
#include "proto/RobotClientsService.grpc.pb.h"

class CloudExchangeStream : public grpc::ClientBidiReactor<RobotClientsExchange, RobotClientsRespond>
{
  private:
    std::shared_ptr<CloudSignals> cloudSignals;
    grpc::ClientContext context;
    RobotClientsExchange *requestPtr;
    RobotClientsRespond respond;
    std::mutex mutex;
    std::condition_variable cv;
    grpc::Status finalStatus;
    bool isShutdown = false;
    bool done = false;

    void OnWriteDone(bool ok) override;
    void OnReadDone(bool ok) override;
    void OnDone(const grpc::Status& status) override;

  public:
    CloudExchangeStream(RobotClientsService::Stub *stub, 
      std::shared_ptr<grpc::CallCredentials> accessToken,
      std::shared_ptr<CloudSignals> cloudSignalsPtr);
    void SendMessage(RobotClientsRobotStatus robotStatus,
      RobotClientsRobotCriticalStatus &criticalStatus,
      std::vector<double> &batteries,
      RobotClientsDof &position,
      RobotClientsAutoTaskNavProgress &navProgress);
    void Shutdown();
    grpc::Status AwaitCompletion();
};

#endif // CLOUDEXCHANGESTREAM_HPP