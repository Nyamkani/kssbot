#ifndef KSSBOT_DIFFDRIVE_CONFIG_H
#define KSSBOT_DIFFDRIVE_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30;

};


#endif // KSSBOT_DIFFDRIVE_CONFIG_H