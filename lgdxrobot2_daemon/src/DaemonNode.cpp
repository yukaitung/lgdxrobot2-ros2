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

  grpc::ClientContext *context = new grpc::ClientContext();
  RpcCompleteToken request;
  request.set_taskid(1);
  request.set_token("");
  RpcResultMessage *respond = new RpcResultMessage();
  std::cout << "send" << std::endl;
  grpcStub->async()->AbortAutoTask(context, &request, respond, [context, respond, this](grpc::Status status){
    if (!status.ok()) {
      std::cout << " rpc failed." << std::endl;
    }
    else {
      std::cout << "rpc ok" << std::endl;
    }
    delete context;
    delete respond;
  });
  std::cout << "send ok" << std::endl;

  grpc::ClientContext *context2 = new grpc::ClientContext();
  RpcRobotExchangeData request2 = MakeExchangeData();
  RpcResultMessageWithTask *respond2 = new RpcResultMessageWithTask();
  std::cout << "send" << std::endl;
  grpcStub->async()->Exchange(context2, &request2, respond2, [context2, respond2, this](grpc::Status status){
    if (!status.ok()) {
      std::cout << " rpc failed." << std::endl;
    }
    else {
      std::cout << "rpc ok" << std::endl;
    }
    delete context2;
    delete respond2;
  });
  

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