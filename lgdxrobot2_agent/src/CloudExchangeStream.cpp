#include "lgdxrobot2_agent/CloudExchangeStream.hpp"

CloudExchangeStream::CloudExchangeStream(RobotClientsService::Stub *stub, 
  std::shared_ptr<grpc::CallCredentials> accessToken,
  std::shared_ptr<CloudSignals> cloudSignalsPtr)
{
  cloudSignals = cloudSignalsPtr;
  context.set_credentials(accessToken);
  stub->async()->ExchangeStream(&context, this);
  StartRead(&respond);
  StartCall();
}

void CloudExchangeStream::OnWriteDone(bool ok)
{
  if (ok)
  {
    cloudSignals->NextExchange();
    if (requestPtr != nullptr)
    {
      delete requestPtr;
      requestPtr = nullptr;
    }
  }
}

void CloudExchangeStream::OnReadDone(bool ok)
{
  if (ok && !isShutdown)
  {
    cloudSignals->HandleExchange(&respond); 
    StartRead(&respond);
  }
}

void CloudExchangeStream::OnDone(const grpc::Status& status)
{
  std::lock_guard<std::mutex> lock(mutex);
  finalStatus = status;
  done = true;
  cv.notify_one();
}

void CloudExchangeStream::SendMessage(RobotClientsRobotStatus robotStatus,
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

void CloudExchangeStream::Shutdown()
{
  isShutdown = true;
  StartWritesDone();
}

grpc::Status CloudExchangeStream::AwaitCompletion()
{
  std::unique_lock<std::mutex> lock(mutex);
  cv.wait(lock, [this] { return done; });
  return std::move(finalStatus);
}