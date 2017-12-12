#ifndef NODEAVAILABLE_H
#define NODEAVAILABLE_H

#include "../configinterface.h"

class NodeAvailable : public ConfigInterface
{
public:
  NodeAvailable();

  void check(ros_monitoring::KeyValue newMsg);
private:
  float errorlevel;

};

#endif // NODEAVAILABLE_H
