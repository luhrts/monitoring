#include "nodeavailable.h"

NodeAvailable::NodeAvailable(errorlevel, ros::Publisher pub)
  :ConfigInterface(pub)
  ,errorlevel(errorlevel)
{

}

void NodeAvailable::check(ros_monitoring::KeyValue newMsg)
{
  if(newMsg.key == "node unavailable") {
    ros_monitoring::Error errormsg;
    errormsg.key = "restart node";
    errormsg.value = newMsg.value;
    errormsg.level = errorlevel;
    publishError(errormsg);
  }
}
