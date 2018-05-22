#include "monitoring_fdir/fdi/std_validator/nodeavailable.h"

NodeAvailable::NodeAvailable(float errorwert, ros::Publisher pub)
  :ConfigInterface(pub)
  ,errorlevel(errorwert)
{

}

/**
 * redirects the keyvalue pair if the msg node unavailable is read, so the recovery system can restart it.
 */
void NodeAvailable::check(monitoring_msgs::KeyValue newMsg)
{
  if(newMsg.key == "node unavailable") {
    monitoring_msgs::Error errormsg;
    errormsg.header.stamp = ros::Time::now();
    errormsg.key = "node dead";
    errormsg.value = newMsg.value;
    errormsg.unit = newMsg.unit;
    errormsg.errorlevel = errorlevel;
    publishError(errormsg);
  }
}
