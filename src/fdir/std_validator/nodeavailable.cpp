#include "nodeavailable.h"

NodeAvailable::NodeAvailable(float errorwert, ros::Publisher pub)
  :ConfigInterface(pub)
  ,errorlevel(errorwert)
{

}

void NodeAvailable::check(ros_monitoring::KeyValue newMsg)
{
  if(newMsg.key == "node unavailable") {
    ros_monitoring::Error errormsg;
    errormsg.key = "node dead";
    errormsg.value = newMsg.value;
    errormsg.level = errorlevel;
    publishError(errormsg);
  }
}
