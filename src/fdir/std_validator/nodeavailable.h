#ifndef NODEAVAILABLE_H
#define NODEAVAILABLE_H

#include "../configinterface.h"

class NodeAvailable : public ConfigInterface
{
public:
  NodeAvailable(float errorwert, ros::Publisher pub);

  void check(ros_monitoring::KeyValue newMsg);
private:
  float errorlevel;

};

#endif // NODEAVAILABLE_H
