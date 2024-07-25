#include "CloudAdapter.hpp"

#include <cstdio>
#include <fstream>
#include <chrono>

#include "hwinfo/hwinfo.h"
#include "hwinfo/utils/unit.h"
#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"

CloudAdapter::CloudAdapter(const char *serverAddress,
  const char *rootCertPath,
  const char *clientCertPath,
  const char *clientKeyPath,
  std::function<void(void)> startNextExchangeCb,
  std::function<void(const RpcRespond *)> updateDaemonCb,
  std::function<void(const char *, int)> logCb,
  std::function<void(CloudFunctions)> errorCb)
{
  startNextExchange = startNextExchangeCb;
  updateDeamon = updateDaemonCb;
  log = logCb;
  error = errorCb;
  std::string rootCert = readCert(rootCertPath);
  std::string clientCert = readCert(clientCertPath);
  std::string clientKey = readCert(clientKeyPath);
  grpc::SslCredentialsOptions sslOptions = {rootCert, clientKey, clientCert};

  grpcChannel = grpc::CreateChannel(serverAddress, grpc::SslCredentials(sslOptions));
  grpcStub = RobotClientService::NewStub(grpcChannel);
}

std::string CloudAdapter::readCert(const char *filename)
{
  std::ifstream file(filename, std::ios::in);
  if (file.is_open())
  {
    std::stringstream ss;
		ss << file.rdbuf();
		file.close();
		return ss.str();
  }
  return {};
}

void CloudAdapter::setSystemInfo(RpcRobotSystemInfo *info)
{
  const auto cpus = hwinfo::getAllCPUs();
  if (cpus.size() > 0)
  {
    hwinfo::CPU cpu = cpus.at(0);
    info->set_cpu(cpu.modelName());
  }
  else
  {
    info->set_cpu("");
  }
  hwinfo::OS os;
  info->set_os(os.name());
  info->set_is32bit(os.is32bit());
  info->set_islittleendian(os.isLittleEndian());
  const auto gpus = hwinfo::getAllGPUs();
  if (gpus.size() > 0)
  {
    hwinfo::GPU gpu = gpus.at(0);
    info->set_gpu(gpu.name());
  }
  hwinfo::Memory memory;
  info->set_rammib(hwinfo::unit::bytes_to_MiB(memory.total_Bytes()));
}

void CloudAdapter::greet()
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RpcGreet *request = new RpcGreet();
  RpcRobotSystemInfo *systemInfo = new RpcRobotSystemInfo();
  setSystemInfo(systemInfo);
  request->set_allocated_systeminfo(systemInfo);

  RpcRespond *respond = new RpcRespond();
  
  grpcStub->async()->Greet(context, request, respond, [context, request, systemInfo, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      log("Connect to the cloud, start data exchange.", 1);
      startNextExchange();
    }
    else
    {
      log("Unable to connect the cloud, will try again later.", 3);
      error(CloudFunctions::Greet);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void CloudAdapter::exchange()
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RpcExchange *request = new RpcExchange();
  request->add_batteries(12.1f);
  request->add_batteries(12.2f);
  request->add_emergencystopsenabled(true);
  request->add_emergencystopsenabled(true);
  request->set_gettask(false);
  RpcRobotDof *position = new RpcRobotDof();
  position->set_x(0);
  position->set_y(0);
  position->set_w(0);
  request->set_allocated_position(position);
  RpcRobotDof *velocity = new RpcRobotDof();
  velocity->set_x(0);
  velocity->set_y(0);
  velocity->set_w(0);
  request->set_allocated_velocity(velocity);

  RpcRespond *respond = new RpcRespond();

  grpcStub->async()->Exchange(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
      startNextExchange();
    }
    else 
    {
      log("CloudAdapter::exchange() failed.", 3);
      error(CloudFunctions::Exchange);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void CloudAdapter::autoTaskNext(RpcCompleteToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RpcCompleteToken *request = new RpcCompleteToken();
  *request = token;

  RpcRespond *respond = new RpcRespond();

  grpcStub->async()->AutoTaskNext(context, request, respond, [context, request,respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
    }
    else 
    {
      log("CloudAdapter::autoTaskNext() failed.", 3);
      error(CloudFunctions::Exchange);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void CloudAdapter::autoTaskAbort(RpcCompleteToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RpcCompleteToken *request = new RpcCompleteToken();
  *request = token;

  RpcRespond *respond = new RpcRespond();

  grpcStub->async()->AutoTaskAbort(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
    }
    else 
    {
      log("CloudAdapter::autoTaskAbort() failed.", 3);
      error(CloudFunctions::AutoTaskAbort);
    }
    delete context;
    delete request;
    delete respond;
  });
}