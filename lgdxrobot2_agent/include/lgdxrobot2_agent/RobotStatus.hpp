#ifndef ROBOTSTATUS_HPP
#define ROBOTSTATUS_HPP

#include "proto/RobotClientsService.pb.h"

class RobotStatus
{
  private:
    RobotClientsRobotStatus robotStatus = RobotClientsRobotStatus::Offline;
    RobotClientsRobotStatus previousRobotStatus = RobotClientsRobotStatus::Offline;
    bool pauseTaskAssignmentFlag = false;

    void ChangeStatus(RobotClientsRobotStatus newState);

  public:
    RobotClientsRobotStatus GetStatus();

    void ConnnectedCloud();
    void StartCharging();
    void ChargingCompleted();
    void TaskAssigned();
    void TaskCompleted();
    void NavigationStuck();
    void NavigationCleared();
    void PauseTaskAssignment();
    void ResumeTaskAssignment();
    void TaskAborting();
    void TaskAborted();
    void EnterCritical();
    void ExitCritical();
};

#endif // ROBOTSTATUS_HPP