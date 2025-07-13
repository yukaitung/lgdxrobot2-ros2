#include "lgdxrobot2_agent/RobotStatus.hpp"

RobotClientsRobotStatus RobotStatus::getRobotStatus()
{
  return robotStatus;
}

void RobotStatus::changeStatus(RobotClientsRobotStatus newState)
{
  switch (newState)
  {
    case RobotClientsRobotStatus::Idle:
      if (pauseTaskAssigementFlag) 
      {
        robotStatus = RobotClientsRobotStatus::Paused;
        return;
        // Exit Function
      }
      break;
    default:
      break;
  }
  robotStatus = newState;
}

void RobotStatus::connnectedCloud()
{
  if (robotStatus == RobotClientsRobotStatus::Offline)
    changeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::startCharging()
{
  if (robotStatus == RobotClientsRobotStatus::Idle)
    changeStatus(RobotClientsRobotStatus::Charging);
}

void RobotStatus::chargingCompleted()
{
  if (robotStatus == RobotClientsRobotStatus::Charging)
    changeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::taskAssigned()
{
  if (robotStatus == RobotClientsRobotStatus::Idle)
    changeStatus(RobotClientsRobotStatus::Running);
}

void RobotStatus::taskCompleted()
{
  if (robotStatus == RobotClientsRobotStatus::Running || robotStatus == RobotClientsRobotStatus::Stuck)
    changeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::navigationStuck()
{
  if (robotStatus == RobotClientsRobotStatus::Running)
    changeStatus(RobotClientsRobotStatus::Stuck);
}

void RobotStatus::navigationCleared()
{
  if (robotStatus == RobotClientsRobotStatus::Stuck)
    changeStatus(RobotClientsRobotStatus::Running);
}

void RobotStatus::pauseTaskAssigement()
{
  pauseTaskAssigementFlag = true;
  if (robotStatus != RobotClientsRobotStatus::Idle)
  {
    // Delay this state if the robot is running
    return;
  }
  changeStatus(RobotClientsRobotStatus::Paused);
}

void RobotStatus::resumeTaskAssigement()
{
  pauseTaskAssigementFlag = false;
  if (robotStatus == RobotClientsRobotStatus::Paused)
  {
    // Change state immediately if the robot is paused
    changeStatus(RobotClientsRobotStatus::Idle);
  }
}

void RobotStatus::taskAborting()
{
  if (robotStatus == RobotClientsRobotStatus::Running || robotStatus == RobotClientsRobotStatus::Stuck)
    changeStatus(RobotClientsRobotStatus::Aborting);
}

void RobotStatus::taskAborted()
{
  if (robotStatus == RobotClientsRobotStatus::Aborting || robotStatus == RobotClientsRobotStatus::Running)
    changeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::enterCritical()
{
  previousRobotStatus = robotStatus;
  changeStatus(RobotClientsRobotStatus::Critical);
}

void RobotStatus::exitCritical()
{
  if (robotStatus == RobotClientsRobotStatus::Critical)
    changeStatus(previousRobotStatus);
}