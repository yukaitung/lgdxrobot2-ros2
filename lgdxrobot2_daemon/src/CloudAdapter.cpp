#include "lgdxrobot2_daemon/CloudAdapter.hpp"

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
  const char *clientKeyPath,
  const char *clientCertPath,
  std::function<void(void)> startNextExchangeCb,
  std::function<void(const RobotClientsRespond *)> updateDaemonCb,
  std::function<void(const char *, int)> logCb,
  std::function<void(CloudFunctions)> errorCb)
{
  startNextExchange = startNextExchangeCb;
  updateDeamon = updateDaemonCb;
  log = logCb;
  error = errorCb;
  std::string rootCert = readCert(rootCertPath);
  std::string clientKey = readCert(clientKeyPath);
  std::string clientCert = readCert(clientCertPath);
  grpc::SslCredentialsOptions sslOptions = {rootCert, clientKey, clientCert};

  grpcChannel = grpc::CreateChannel(serverAddress, grpc::SslCredentials(sslOptions));
  grpcStub = RobotClientsService::NewStub(grpcChannel);
  accessToken = grpc::AccessTokenCredentials("");
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

#ifdef __linux__ 
std::string CloudAdapter::getMotherBoardSerialNumber()
{
  std::ifstream file("/sys/class/dmi/id/board_serial", std::ios::in);
  std::string serialNumber;
  if (file.is_open())
  {
    std::getline(file, serialNumber);
  }
  else
  {
    log("Unable to read motherboard serial number.", 3);
  }
  return serialNumber;
}
#endif

void CloudAdapter::setSystemInfo(RobotClientsSystemInfo *info)
{
  hwinfo::MainBoard main_board;
  info->set_motherboard(main_board.name());
  #ifdef __linux__ 
    info->set_motherboardserialnumber(getMotherBoardSerialNumber());
  #else
    info->set_motherboardserialnumber(main_board.serialNumber());
  #endif
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

  // TODO: Get MCU serial number
}

void CloudAdapter::greet()
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RobotClientsGreet *request = new RobotClientsGreet();
  RobotClientsSystemInfo *systemInfo = new RobotClientsSystemInfo();
  setSystemInfo(systemInfo);
  request->set_allocated_systeminfo(systemInfo);

  RobotClientsGreetRespond *respond = new RobotClientsGreetRespond();
  
  grpcStub->async()->Greet(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      accessToken = grpc::AccessTokenCredentials(respond->accesstoken());
      // TODO: Store ChassisInfo
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

void CloudAdapter::exchange(RobotClientsRobotStatus robotStatus,
  RobotClientsRobotCriticalStatus &criticalStatus,
  std::vector<double> &batteries,
  RobotClientsDof &position,
  RobotClientsAutoTaskNavProgress &navProgress)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);
  context->set_credentials(accessToken);

  RobotClientsExchange *request = new RobotClientsExchange();
  request->set_robotstatus(robotStatus);
  RobotClientsRobotCriticalStatus *intCriticalStatus = new RobotClientsRobotCriticalStatus();
  *intCriticalStatus = criticalStatus;
  request->set_allocated_criticalstatus(intCriticalStatus);
  for (auto &battery : batteries)
  {
    request->add_batteries(battery);
  }
  RobotClientsDof *intPosition = new RobotClientsDof();
  *intPosition = position;
  request->set_allocated_position(intPosition);
  RobotClientsAutoTaskNavProgress *intNavProgress = new RobotClientsAutoTaskNavProgress();
  *intNavProgress = navProgress;
  request->set_allocated_navprogress(intNavProgress);

  RobotClientsRespond *respond = new RobotClientsRespond();

  grpcStub->async()->Exchange(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
      startNextExchange();
    }
    else 
    {
      log("Data exchange failed.", 3);
      error(CloudFunctions::Exchange);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void CloudAdapter::autoTaskNext(RobotClientsNextToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);
  context->set_credentials(accessToken);

  RobotClientsNextToken *request = new RobotClientsNextToken();
  *request = token;

  RobotClientsRespond *respond = new RobotClientsRespond();

  grpcStub->async()->AutoTaskNext(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
    }
    else 
    {
      log("AutoTaskNext failed.", 3);
      error(CloudFunctions::Exchange);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void CloudAdapter::autoTaskAbort(RobotClientsAbortToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);
  context->set_credentials(accessToken);

  RobotClientsAbortToken *request = new RobotClientsAbortToken();
  *request = token;

  RobotClientsRespond *respond = new RobotClientsRespond();

  grpcStub->async()->AutoTaskAbort(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      updateDeamon(respond);
    }
    else 
    {
      log("AutoTaskAbort failed.", 3);
      error(CloudFunctions::AutoTaskAbort);
    }
    delete context;
    delete request;
    delete respond;
  });
}