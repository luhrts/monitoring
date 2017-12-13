#include "restartnodehandler.h"

RestartNodeHandler::RestartNodeHandler()
{

}


void RestartNodeHandler::checkError(ros_monitoring::Error msg) {
  char cmd[80];
  sprintf(cmd, "killall %s", msg.value.c_str());
  system(cmd);
}
