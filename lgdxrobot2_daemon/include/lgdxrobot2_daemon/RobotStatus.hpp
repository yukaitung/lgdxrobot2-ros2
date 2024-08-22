#ifndef ROBOTSTATUS_HPP
#define ROBOTSTATUS_HPP

#include "proto/RobotClientsService.pb.h"

class RobotStatus
{
  private:
    RobotClientsRobotStatus robotStatus = RobotClientsRobotStatus::Offline;
    RobotClientsRobotStatus previousRobotStatus = RobotClientsRobotStatus::Offline;
    bool stopTaskAssigementFlag = false;

  public:
    RobotClientsRobotStatus getRobotStatus();

    void connnectedCloud();
    void startCharging();
    void chargingCompleted();
    void taskAssigned();
    void taskCompleted();
    void navigationStuck();
    void navigationCleared();
    void stopTaskAssigement();
    void resumeTaskAssigement();
    void taskAborting();
    void taskAborted();
    void enterCritical();
    void exitCritical();
};

#endif // ROBOTSTATUS_HPP