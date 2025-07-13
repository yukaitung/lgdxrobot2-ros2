#include "lgdxrobot2_agent/RobotStatus.hpp"

RobotClientsRobotStatus RobotStatus::GetStatus()
{
  return robotStatus;
}

void RobotStatus::ChangeStatus(RobotClientsRobotStatus newState)
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

void RobotStatus::ConnnectedCloud()
{
  if (robotStatus == RobotClientsRobotStatus::Offline)
    ChangeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::StartCharging()
{
  if (robotStatus == RobotClientsRobotStatus::Idle)
    ChangeStatus(RobotClientsRobotStatus::Charging);
}

void RobotStatus::ChargingCompleted()
{
  if (robotStatus == RobotClientsRobotStatus::Charging)
    ChangeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::TaskAssigned()
{
  if (robotStatus == RobotClientsRobotStatus::Idle)
    ChangeStatus(RobotClientsRobotStatus::Running);
}

void RobotStatus::TaskCompleted()
{
  if (robotStatus == RobotClientsRobotStatus::Running || robotStatus == RobotClientsRobotStatus::Stuck)
    ChangeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::NavigationStuck()
{
  if (robotStatus == RobotClientsRobotStatus::Running)
    ChangeStatus(RobotClientsRobotStatus::Stuck);
}

void RobotStatus::NavigationCleared()
{
  if (robotStatus == RobotClientsRobotStatus::Stuck)
    ChangeStatus(RobotClientsRobotStatus::Running);
}

void RobotStatus::PauseTaskAssigement()
{
  pauseTaskAssigementFlag = true;
  if (robotStatus != RobotClientsRobotStatus::Idle)
  {
    // Delay this state if the robot is running
    return;
  }
  ChangeStatus(RobotClientsRobotStatus::Paused);
}

void RobotStatus::ResumeTaskAssigement()
{
  pauseTaskAssigementFlag = false;
  if (robotStatus == RobotClientsRobotStatus::Paused)
  {
    // Change state immediately if the robot is paused
    ChangeStatus(RobotClientsRobotStatus::Idle);
  }
}

void RobotStatus::TaskAborting()
{
  if (robotStatus == RobotClientsRobotStatus::Running || robotStatus == RobotClientsRobotStatus::Stuck)
    ChangeStatus(RobotClientsRobotStatus::Aborting);
}

void RobotStatus::TaskAborted()
{
  if (robotStatus == RobotClientsRobotStatus::Aborting || robotStatus == RobotClientsRobotStatus::Running)
    ChangeStatus(RobotClientsRobotStatus::Idle);
}

void RobotStatus::EnterCritical()
{
  previousRobotStatus = robotStatus;
  ChangeStatus(RobotClientsRobotStatus::Critical);
}

void RobotStatus::ExitCritical()
{
  if (robotStatus == RobotClientsRobotStatus::Critical)
    ChangeStatus(previousRobotStatus);
}