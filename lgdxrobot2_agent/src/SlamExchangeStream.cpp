#include "lgdxrobot2_agent/SlamExchangeStream.hpp"

SlamExchangeStream::SlamExchangeStream(RobotClientsService::Stub *stub, 
  std::shared_ptr<grpc::CallCredentials> accessToken,
  std::shared_ptr<CloudSignals> cloudSignalsPtr)
{
  requestPtr = new RobotClientsSlamExchange();

  cloudSignals = cloudSignalsPtr;
  context.set_credentials(accessToken);
  stub->async()->SlamExchange(&context, this);
  StartRead(&respond);
  StartCall();
}

void SlamExchangeStream::OnWriteDone(bool ok)
{
  if (ok)
  {
    cloudSignals->NextExchange();
  }
}

void SlamExchangeStream::OnReadDone(bool ok)
{
  if (ok && !isShutdown)
  {
    cloudSignals->HandleSlamExchange(&respond); 
    StartRead(&respond);
  }
}

void SlamExchangeStream::OnDone(const grpc::Status& status)
{
  std::lock_guard<std::mutex> lock(mutex);
  finalStatus = status;
  done = true;
  cv.notify_one();
}

void SlamExchangeStream::SendMessage(RobotClientsRealtimeNavResults status,
  RobotClientsExchange &exchange)
{
  if (isShutdown)
  {
    return;
  }

  requestPtr->set_status(status);
  requestPtr->mutable_exchange()->CopyFrom(exchange);
  requestPtr->mutable_mapdata()->Clear();

  StartWrite(requestPtr);
}

void SlamExchangeStream::SendMessage(RobotClientsRealtimeNavResults status,
  RobotClientsExchange &exchange,
  RobotClientsMapData &mapData)
{
  if (isShutdown)
  {
    return;
  }

  requestPtr->set_status(status);
  requestPtr->mutable_exchange()->CopyFrom(exchange);
  requestPtr->mutable_mapdata()->CopyFrom(mapData);

  StartWrite(requestPtr);
}

void SlamExchangeStream::Shutdown()
{
  isShutdown = true;
  StartWritesDone();

  if (requestPtr != nullptr)
  {
    delete requestPtr;
    requestPtr = nullptr;
  }
}

grpc::Status SlamExchangeStream::AwaitCompletion()
{
  std::unique_lock<std::mutex> lock(mutex);
  cv.wait(lock, [this] { return done; });
  return std::move(finalStatus);
}