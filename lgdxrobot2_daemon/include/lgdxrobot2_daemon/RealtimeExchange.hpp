#ifndef REALTIMEEXCHANGE_HPP
#define REALTIMEEXCHANGE_HPP

#include <grpcpp/grpcpp.h>
#include <mutex>
#include <condition_variable>

#include "proto/RobotClientsService.grpc.pb.h"

class RealtimeExchange : public grpc::ClientBidiReactor<RobotClientsExchange, RobotClientsRespond>
{
  private:
    grpc::ClientContext context;
    RobotClientsExchange *requestPtr;
    RobotClientsRespond respond;
    std::mutex mutex;
    std::condition_variable cv;
    grpc::Status finalStatus;
    bool isShutdown = false;
    bool done = false;
    std::function<void(const RobotClientsRespond *)> updateDeamon;
    std::function<void(void)> startNextExchange;

    void OnWriteDone(bool ok) override;
    void OnReadDone(bool ok) override;
    void OnDone(const grpc::Status& status) override;

  public:
    RealtimeExchange(RobotClientsService::Stub *stub, 
      std::shared_ptr<grpc::CallCredentials> accessToken,
      std::function<void(const RobotClientsRespond *)> updateDaemonCb,
      std::function<void(void)> startNextExchangeCb);
    void SendMessage(RobotClientsRobotStatus robotStatus,
      RobotClientsRobotCriticalStatus &criticalStatus,
      std::vector<double> &batteries,
      RobotClientsDof &position,
      RobotClientsAutoTaskNavProgress &navProgress);
    void Shutdown();
    grpc::Status AwaitCompletion();
};

#endif // REALTIMEEXCHANGE_HPP