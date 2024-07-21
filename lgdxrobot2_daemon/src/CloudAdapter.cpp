#include "CloudAdapter.hpp"

#include <iostream>
#include <fstream>

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
  std::function<void(const RpcRespond *)> respondCb,
  std::function<void(const char *, int)> debugCb)
{
  respondCallback = respondCb;
  debugCallback = debugCb;
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

void CloudAdapter::greet(RpcGreet &greet)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  RpcRespond *respond = new RpcRespond();
  grpcStub->async()->Greet(context, &greet, respond, [context, respond, this](grpc::Status status)
  {
    if (!status.ok()) 
    {
      debugCallback("CloudAdapter::greet failed.", 3);
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
      respondCallback(respond);
    }
    else 
    {
      debugCallback("CloudAdapter::exchange failed.", 3);
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
      respondCallback(respond);
    }
    else 
    {
      debugCallback("CloudAdapter::autoTaskNext failed.", 3);
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
      respondCallback(respond);
    }
    else 
    {
      debugCallback("CloudAdapter::autoTaskAbort failed.", 3);
    }
    delete context;
    delete respond;
  });
}