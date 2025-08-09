#include "lgdxrobot2_agent/CloudExchangeStream.hpp"

CloudExchangeStream::CloudExchangeStream(RobotClientsService::Stub *stub, 
  std::shared_ptr<grpc::CallCredentials> accessToken,
  std::shared_ptr<CloudSignals> cloudSignalsPtr)
{
  requestPtr = new RobotClientsExchange();

  cloudSignals = cloudSignalsPtr;
  context.set_credentials(accessToken);
  stub->async()->Exchange(&context, this);
  StartRead(&respond);
  StartCall();
}

CloudExchangeStream::~CloudExchangeStream()
{
  if (requestPtr != nullptr)
  {
    delete requestPtr;
    requestPtr = nullptr;
  }
}

void CloudExchangeStream::OnWriteDone(bool ok)
{
  if (ok && !isShutdown)
  {
    cloudSignals->NextExchange();
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
  if (!status.ok())
  {
    isShutdown = true;
    cloudSignals->StreamError(CloudFunctions::Exchange);
    return;
  }
  std::lock_guard<std::mutex> lock(mutex);
  finalStatus = status;
  done = true;
  cv.notify_one();
}

void CloudExchangeStream::SendMessage(RobotClientsExchange &exchange)
{
  if (isShutdown)
  {
    return;
  }
  *requestPtr = exchange;
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