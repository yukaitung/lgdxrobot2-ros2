#include "DaemonNode.hpp"

#include <iostream>
#include <fstream>

#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"

using namespace std::chrono_literals;

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

DaemonNode::DaemonNode() : Node("lgdxrobot2_daemon_node")
{
  std::string root = readCert("/home/user/key/rootCA.crt");
  std::string cert = readCert("/home/user/key/c1.crt");
  std::string key = readCert("/home/user/key/c1.key");
  grpc::SslCredentialsOptions sslOptions = {root, key, cert};

  std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel("192.168.1.10:5162", grpc::SslCredentials(sslOptions));
  grpcStub = RobotClientService::NewStub(channel);

/*
  grpc::ClientContext context;
  RpcCompleteToken request;
  request.set_taskid(1);
  request.set_token("");
  RpcResultMessage respond;
  std::cout << "send" << std::endl;
  grpc::Status status = grpcStub->AbortAutoTask(&context, request, &respond);
  if (!status.ok()) {
    std::cout << " rpc failed." << std::endl;
  }
  else {
    std::cout << "rpc ok" << std::endl;
  }
  std::cout << "send ok" << std::endl;*/

  grpc::ClientContext context;
  RpcRobotExchangeData request = MakeExchangeData();
  RpcResultMessageWithTask respond;
  std::cout << "send" << std::endl;
  grpc::Status status = grpcStub->Exchange(&context, request, &respond);
  if (!status.ok()) {
    std::cout << " rpc failed." << std::endl;
  }
  else {
    std::cout << "rpc ok" << std::endl;
  }
  std::cout << "send ok" << std::endl;
  

  //autoTaskPublisher = this->create_publisher<lgdxrobot2_daemon::msg::AutoTask>("/daemon/autotask", rclcpp::SensorDataQoS().reliable());
  //autoTaskPublisherTimer = this->create_wall_timer(20ms, std::bind(&DaemonNode::autoTaskPublisherTimerCallback, this));
}

std::string DaemonNode::readCert(const char *filename)
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

void DaemonNode::autoTaskPublisherTimerCallback()
{
  autoTaskPublisher->publish(currentTask);
}