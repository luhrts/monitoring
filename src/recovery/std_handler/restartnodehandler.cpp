#include "restartnodehandler.h"

RestartNodeHandler::RestartNodeHandler()
{

}

/**
 * Restarts the node by killing it. The launchfile should be on respawn=true, therefore the node gets restarted.
 */
void RestartNodeHandler::checkError(ros_monitoring::Error msg) {
  char cmd[80];
  sprintf(cmd, "killall %s", msg.value.c_str());
  system(cmd);
}
