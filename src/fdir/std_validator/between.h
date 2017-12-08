#ifndef BETWEEN_H
#define BETWEEN_H

#include "../configinterface.h"

class Between : public ConfigInterface
{
public:
  Between(float maxValue, std::string maxErrorMsg, float maxerrorLevel, float minValue, std::string minErrorMsg, float minerrorLevel, ros::Publisher& publisher);
  ~Between();

  void check(ros_monitoring::KeyValue newMsg);

 private:
  float maxValue, maxlevel, minValue, minlevel;
  std::string maxmsg, minmsg;
};

#endif // BETWEEN_H
