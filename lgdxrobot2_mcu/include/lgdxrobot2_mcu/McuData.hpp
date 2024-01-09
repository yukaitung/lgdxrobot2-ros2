#ifndef MCUDATA_HPP
#define MCUDATA_HPP

struct McuData{
  const int wheelCount = 4;
  float targetWheelVelocity[4] = {0};
  float measuredWheelVelocity[4] = {0};
  float pConstant[4] = {0};
  float iConstant[4] = {0};
  float dConstant[4] = {0};
  float battery[2] = {0};
  int estop[2] = {0};
};

#endif // MCUDATA_HPP
