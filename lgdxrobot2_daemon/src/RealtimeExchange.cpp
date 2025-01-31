#include "lgdxrobot2_daemon/RealtimeExchange.hpp"

RealtimeExchange::RealtimeExchange(RobotClientsService::Stub *stub, 
  std::shared_ptr<grpc::CallCredentials> accessToken,
  std::function<void(const RobotClientsRespond *)> updateDaemonCb,
  std::function<void(void)> startNextExchangeCb)
{
  updateDeamon = updateDaemonCb;
  startNextExchange = startNextExchangeCb;
  context.set_credentials(accessToken);
  stub->async()->ExchangeStream(&context, this);
  StartRead(&respond);
  StartCall();
}

void RealtimeExchange::OnWriteDone(bool ok)
{
  if (ok)
  {
    startNextExchange();
    if (requestPtr != nullptr)
    {
      delete requestPtr;
      requestPtr = nullptr;
    }
  }
}

void RealtimeExchange::OnReadDone(bool ok)
{
  if (ok && !isShutdown)
  {
    updateDeamon(&respond);    
    StartRead(&respond);
  }
}

void RealtimeExchange::OnDone(const grpc::Status& status)
{
  std::lock_guard<std::mutex> lock(mutex);
  finalStatus = status;
  done = true;
  cv.notify_one();
}

void RealtimeExchange::SendMessage(RobotClientsRobotStatus robotStatus,
  RobotClientsRobotCriticalStatus &criticalStatus,
  std::vector<double> &batteries,
  RobotClientsDof &position,
  RobotClientsAutoTaskNavProgress &navProgress)
{
  if (isShutdown)
  {
    return;
  }

  requestPtr = new RobotClientsExchange();
  requestPtr->set_robotstatus(robotStatus);
  RobotClientsRobotCriticalStatus *intCriticalStatus = new RobotClientsRobotCriticalStatus();
  *intCriticalStatus = criticalStatus;
  requestPtr->set_allocated_criticalstatus(intCriticalStatus);
  for (auto &battery : batteries)
  {
    requestPtr->add_batteries(battery);
  }
  RobotClientsDof *intPosition = new RobotClientsDof();
  *intPosition = position;
  requestPtr->set_allocated_position(intPosition);
  RobotClientsAutoTaskNavProgress *intNavProgress = new RobotClientsAutoTaskNavProgress();
  *intNavProgress = navProgress;
  requestPtr->set_allocated_navprogress(intNavProgress);
  
  StartWrite(requestPtr);
}

void RealtimeExchange::Shutdown()
{
  isShutdown = true;
  StartWritesDone();
}

grpc::Status RealtimeExchange::AwaitCompletion()
{
  std::unique_lock<std::mutex> lock(mutex);
  cv.wait(lock, [this] { return done; });
  return std::move(finalStatus);
}