#ifndef MIN_H
#define MIN_H

#include "../configinterface.h"

class Min : public ConfigInterface
{
public:
  Min(float value, std::string errormsg, float errorLevel, ros::Publisher& publisher);
  ~Min();
  void check(monitoring_msgs::KeyValue newMsg);

private:
  float minValue, errorlevel;
  std::string msg;
};

#endif // MIN_H
