#include "CloudAdapter.hpp"

#include <iostream>
#include <fstream>

#include "hwinfo/hwinfo.h"
#include "hwinfo/utils/unit.h"
#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"

/*
RpcRobotDof MakeRobotDof()
{
  RpcRobotDof dof;
  dof.set_x(0);
  dof.set_y(0);
  dof.set_w(0);
  return dof;
}

RpcRobotExchangeData MakeExchangeData()
{
  RpcRobotExchangeData data;
  data.add_batteries(12.1f);
  data.add_batteries(12.2f);
  data.add_emergencystopsenabled(true);
  data.add_emergencystopsenabled(true);
  data.set_gettask(false);
  RpcRobotDof *position = new RpcRobotDof();
  *position = MakeRobotDof();
  RpcRobotDof *velocity = new RpcRobotDof();
  *velocity = MakeRobotDof();
  data.set_allocated_position(position);
  data.set_allocated_velocity(velocity);
  return data;
}
*/

CloudAdapter::CloudAdapter (const char *serverAddress,
  const char *rootCertPath,
  const char *clientCertPath,
  const char *clientKeyPath,
  std::function<void(const RpcRespond *)> updateDaemonCb,
  std::function<void(const char *, int)> logCb)
{
  updateDeamon = updateDaemonCb;
  log = logCb;
  std::string rootCert = readCert(rootCertPath);
  std::string clientCert = readCert(clientCertPath);
  std::string clientKey = readCert(clientKeyPath);
  grpc::SslCredentialsOptions sslOptions = {rootCert, clientKey, clientCert};

  grpcChannel = grpc::CreateChannel(serverAddress, grpc::SslCredentials(sslOptions));
  grpcStub = RobotClientService::NewStub(grpcChannel);

  systemInfo = GenerateSystemInfo();
  greet = GenerateGreet(&systemInfo);
  grpcGreet(greet);
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

RpcRobotSystemInfo CloudAdapter::GenerateSystemInfo()
{
  RpcRobotSystemInfo info;
  const auto cpus = hwinfo::getAllCPUs();
  if (cpus.size() > 0)
  {
    hwinfo::CPU cpu = cpus.at(0);
    info.set_cpu(cpu.modelName());
  }
  else
  {
    info.set_cpu("");
  }
  hwinfo::OS os;
  info.set_os(os.name());
  info.set_is32bit(os.is32bit());
  info.set_islittleendian(os.isLittleEndian());
  const auto gpus = hwinfo::getAllGPUs();
  if (gpus.size() > 0)
  {
    hwinfo::GPU gpu = gpus.at(0);
    info.set_gpu(gpu.name());
  }
  hwinfo::Memory memory;
  info.set_rammib(hwinfo::unit::bytes_to_MiB(memory.total_Bytes()));
  return info;
}

RpcGreet CloudAdapter::GenerateGreet(RpcRobotSystemInfo *systemInfo)
{
  RpcGreet greet;
  greet.set_allocated_systeminfo(systemInfo);
  return greet;
}

void CloudAdapter::grpcGreet(RpcGreet &greet)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  RpcRespond *respond = new RpcRespond();
  grpcStub->async()->Greet(context, &greet, respond, [context, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      log("grpcGreet() succeed, start data exchange.", 1);
    }
    else
    {
      log("CloudAdapter::greet failed.", 3);
    }
    delete context;
    delete respond;
  });
}

void CloudAdapter::exchange(RpcExchange &exchange)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  RpcRespond *respond = new RpcRespond();
  grpcStub->async()->Exchange(context, &exchange, respond, [context, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
    }
    else 
    {
      log("CloudAdapter::exchange failed.", 3);
    }
    delete context;
    delete respond;
  });
}

void CloudAdapter::autoTaskNext(RpcCompleteToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  RpcRespond *respond = new RpcRespond();
  grpcStub->async()->AutoTaskNext(context, &token, respond, [context, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
    }
    else 
    {
      log("CloudAdapter::autoTaskNext failed.", 3);
    }
    delete context;
    delete respond;
  });
}

void CloudAdapter::autoTaskAbort(RpcCompleteToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  RpcRespond *respond = new RpcRespond();
  grpcStub->async()->AutoTaskAbort(context, &token, respond, [context, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
    }
    else 
    {
      log("CloudAdapter::autoTaskAbort failed.", 3);
    }
    delete context;
    delete respond;
  });
}