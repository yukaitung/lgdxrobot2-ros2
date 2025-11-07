#ifndef ROBOTDATA_HPP
#define ROBOTDATA_HPP

#include <cstdint>

struct RobotData
{
  const int wheelCount = 4;
  uint32_t responseTime = 0;
  float transform[3] = {0};
  float forwardKinematic[3] = {0};
  float motorsTargetVelocity[4] = {0};
  float motorsDesireVelocity[4] = {0};
  float motorsActualVelocity[4] = {0};
  float motorsCcr[4] = {0};
  float batteryCurrent[2] = {0};
  float batteryVoltage[2] = {0};
  int eStop[3] = {0};
};

#endif // ROBOTDATA_HPP
