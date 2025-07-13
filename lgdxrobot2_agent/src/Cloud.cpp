#include "lgdxrobot2_agent/Cloud.hpp"

#include <random>
#include <fstream>

#include "hwinfo/hwinfo.h"
#include "hwinfo/utils/unit.h"
#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"

Cloud::Cloud(rclcpp::Node::SharedPtr node,
    std::shared_ptr<CloudSignals> cloudSignalsPtr,
    std::shared_ptr<RobotStatus> robotStatusPtr
  ) : logger_(node->get_logger())
{
  cloudSignals = cloudSignalsPtr;
  robotStatus = robotStatusPtr;

  // Parameters
  auto cloudAddressParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudAddressParam.description = "Address of LGDXRobot2 Cloud.";
  node->declare_parameter("cloud_address", "", cloudAddressParam);
  auto cloudRootCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudRootCertParam.description = "Path to server root certificate, required in LGDXRobot2 Cloud.";
  node->declare_parameter("cloud_root_cert", "", cloudRootCertParam);
  auto cloudClientKeyParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientKeyParam.description = "Path to client's private key, required in LGDXRobot2 Cloud.";
  node->declare_parameter("cloud_client_key", "", cloudClientKeyParam);
  auto cloudClientCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientCertParam.description = "Path to client's certificate chain, required in LGDXRobot2 Cloud.";
  node->declare_parameter("cloud_client_cert", "", cloudClientCertParam);

  // Cloud Initalise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1000, 6000);
  int cloudRetryWait = dis(gen);
  RCLCPP_INFO(logger_, "LGDXRobot Cloud is enabled, the break for reconnection to the cloud is %d ms.", cloudRetryWait);
  cloudRetryTimer = node->create_wall_timer(std::chrono::milliseconds(cloudRetryWait), 
    std::bind(&Cloud::HandleError, this));
  cloudRetryTimer->cancel();
  
  // Connect to Cloud
  std::string serverAddress = node->get_parameter("cloud_address").as_string();
  std::string rootCertPath = node->get_parameter("cloud_root_cert").as_string();
  std::string clientKeyPath = node->get_parameter("cloud_client_key").as_string();
  std::string clientCertPath = node->get_parameter("cloud_client_cert").as_string();
  std::string rootCert = ReadCertificate(rootCertPath.c_str());
  std::string clientKey = ReadCertificate(clientKeyPath.c_str());
  std::string clientCert = ReadCertificate(clientCertPath.c_str());
  grpc::SslCredentialsOptions sslOptions = {rootCert, clientKey, clientCert};

  grpcChannel = grpc::CreateChannel(serverAddress, grpc::SslCredentials(sslOptions));
  grpcStub = RobotClientsService::NewStub(grpcChannel);
  accessToken = grpc::AccessTokenCredentials("");
}

std::string Cloud::ReadCertificate(const char *filename)
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
std::string Cloud::GetMotherBoardSerialNumber()
{
  std::ifstream file("/sys/class/dmi/id/board_serial", std::ios::in);
  std::string serialNumber;
  if (file.is_open())
  {
    std::getline(file, serialNumber);
  }
  else
  {
    RCLCPP_ERROR(logger_, "Unable to read motherboard serial number.");
  }
  return serialNumber;
}
#endif

void Cloud::SetSystemInfo(RobotClientsSystemInfo *info)
{
  hwinfo::MainBoard main_board;
  info->set_motherboard(main_board.name());
  #ifdef __linux__ 
    info->set_motherboardserialnumber(GetMotherBoardSerialNumber());
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
}

void Cloud::ExchangePolling(RobotClientsRobotCriticalStatus &criticalStatus,
  std::vector<double> &batteries,
  RobotClientsDof &position,
  RobotClientsAutoTaskNavProgress &navProgress)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);
  context->set_credentials(accessToken);

  RobotClientsExchange *request = new RobotClientsExchange();
  request->set_robotstatus(robotStatus->GetStatus());
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
      cloudSignals->HandleExchange(respond);
      cloudSignals->NextExchange();
    }
    else 
    {
      RCLCPP_ERROR(logger_, "Data exchange failed.");
      Error(CloudFunctions::Exchange);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void Cloud::ExchangeStream(RobotClientsRobotCriticalStatus &criticalStatus,
  std::vector<double> &batteries,
  RobotClientsDof &position,
  RobotClientsAutoTaskNavProgress &navProgress)
{
  if (cloudExchangeStream != nullptr)
  {
    cloudExchangeStream->SendMessage(robotStatus->GetStatus(), criticalStatus, batteries, position, navProgress);
  }
}

void Cloud::Error(CloudFunctions function)
{
  cloudErrorQueue.push(function);
  if (cloudRetryTimer->is_canceled())
    cloudRetryTimer->reset();
}

void Cloud::HandleError()
{
  cloudRetryTimer->cancel();
  // Avoid inf loop if the queue increases
  Greet(cloudErrorRetryData.mcuSerialNumber);
  for (int i = 0, size = cloudErrorQueue.size(); i < size; i++)
  {
    CloudFunctions function = cloudErrorQueue.front();
    switch (function)
    {
    case CloudFunctions::Greet:
      // Do nothing
      break;
    case CloudFunctions::Exchange:
      cloudSignals->NextExchange();
      break;
    case CloudFunctions::AutoTaskNext:
      AutoTaskNext(cloudErrorRetryData.nextToken);
      break;
    case CloudFunctions::AutoTaskAbort:
      AutoTaskAbort(cloudErrorRetryData.abortToken);
      break;
    }
    cloudErrorQueue.pop();
  }
}

void Cloud::Greet(std::string mcuSN)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RobotClientsGreet *request = new RobotClientsGreet();
  RobotClientsSystemInfo *systemInfo = new RobotClientsSystemInfo();
  SetSystemInfo(systemInfo);
  if(!mcuSN.empty())
  {
    systemInfo->set_mcuserialnumber(mcuSN);
  }
  request->set_allocated_systeminfo(systemInfo);

  RobotClientsGreetRespond *respond = new RobotClientsGreetRespond();

  cloudErrorRetryData.mcuSerialNumber = mcuSN;
  
  grpcStub->async()->Greet(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      accessToken = grpc::AccessTokenCredentials(respond->accesstoken());
      RCLCPP_INFO(logger_, "Connect to the cloud, start data exchange.");
      robotStatus->ConnnectedCloud();
      if (respond->isrealtimeexchange())
      {
        isRealtimeExchange = true;
        RCLCPP_INFO(logger_, "Data exchange is realtime.");
        grpcRealtimeStub = RobotClientsService::NewStub(grpcChannel);
        cloudExchangeStream = std::make_unique<CloudExchangeStream>(grpcRealtimeStub.get(), accessToken);
      }
      // Start the timer to exchange data
      cloudSignals->NextExchange();
    }
    else
    {
      RCLCPP_ERROR(logger_, "Unable to connect the cloud, will try again later.");
      Error(CloudFunctions::Greet);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void Cloud::Exchange(RobotClientsRobotCriticalStatus &criticalStatus,
  std::vector<double> &batteries,
  RobotClientsDof &position,
  RobotClientsAutoTaskNavProgress &navProgress)
{
  if (isRealtimeExchange)
  {
    ExchangeStream(criticalStatus, batteries, position, navProgress);
  }
  else
  {
    ExchangePolling(criticalStatus, batteries, position, navProgress);
  }
}

void Cloud::AutoTaskNext(RobotClientsNextToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);
  context->set_credentials(accessToken);

  RobotClientsNextToken *request = new RobotClientsNextToken();
  *request = token;

  RobotClientsRespond *respond = new RobotClientsRespond();

  cloudErrorRetryData.nextToken = token;

  grpcStub->async()->AutoTaskNext(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      cloudSignals->HandleExchange(respond);
    }
    else 
    {
      RCLCPP_ERROR(logger_, "AutoTaskNext failed.");
      Error(CloudFunctions::Exchange);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void Cloud::AutoTaskAbort(RobotClientsAbortToken &token)
{
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);
  context->set_credentials(accessToken);

  RobotClientsAbortToken *request = new RobotClientsAbortToken();
  *request = token;

  RobotClientsRespond *respond = new RobotClientsRespond();

  cloudErrorRetryData.abortToken = token;

  grpcStub->async()->AutoTaskAbort(context, request, respond, [context, request, respond, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      cloudSignals->HandleExchange(respond);
    }
    else 
    {
      RCLCPP_ERROR(logger_, "AutoTaskAbort failed.");
      Error(CloudFunctions::AutoTaskAbort);
    }
    delete context;
    delete request;
    delete respond;
  });
}

void Cloud::Shutdown()
{
  if (cloudExchangeStream != nullptr)
  {
    cloudExchangeStream->Shutdown();
    cloudExchangeStream->AwaitCompletion();
  }
}