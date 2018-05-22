#ifndef NODEAVAILABLE_H
#define NODEAVAILABLE_H

#include "../configinterface.h"

class NodeAvailable : public ConfigInterface
{
public:
  NodeAvailable(float errorwert, ros::Publisher pub);

  void check(monitoring_msgs::KeyValue newMsg);
private:
  float errorlevel;

};

#endif // NODEAVAILABLE_H
