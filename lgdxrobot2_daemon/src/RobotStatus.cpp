#include "lgdxrobot2_daemon/RobotStatus.hpp"

RobotClientsRobotStatus RobotStatus::getRobotStatus()
{
  return robotStatus;
}

void RobotStatus::connnectedCloud()
{
  robotStatus = RobotClientsRobotStatus::Idle;
}

void RobotStatus::startCharging()
{
  robotStatus = RobotClientsRobotStatus::Charging;
}

void RobotStatus::chargingCompleted()
{
  robotStatus = RobotClientsRobotStatus::Idle;
}

void RobotStatus::taskAssigned()
{
  robotStatus = RobotClientsRobotStatus::Running;
}

void RobotStatus::taskCompleted()
{
  if (stopTaskAssigementFlag)
  {
    robotStatus = RobotClientsRobotStatus::Paused;
  }
  else
  {
    robotStatus = RobotClientsRobotStatus::Idle;
  }
}

void RobotStatus::navigationStuck()
{
  robotStatus = RobotClientsRobotStatus::Stuck;
}

void RobotStatus::navigationCleared()
{
  robotStatus = RobotClientsRobotStatus::Running;
}

void RobotStatus::stopTaskAssigement()
{
  if (robotStatus != RobotClientsRobotStatus::Idle)
  {
    stopTaskAssigementFlag = true;
    return;
  }
  robotStatus = RobotClientsRobotStatus::Paused;
}

void RobotStatus::resumeTaskAssigement()
{
  robotStatus = RobotClientsRobotStatus::Idle;
  stopTaskAssigementFlag = false;
}

void RobotStatus::taskAborting()
{
  robotStatus = RobotClientsRobotStatus::Aborting;
}

void RobotStatus::taskAborted()
{
  robotStatus = RobotClientsRobotStatus::Idle;
}

void RobotStatus::enterCritical()
{
  previousRobotStatus = robotStatus;
  robotStatus = RobotClientsRobotStatus::Critical;
}

void RobotStatus::exitCritical()
{
  robotStatus = previousRobotStatus;
}