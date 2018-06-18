#include "monitoring_fdir/recovery/std_handler/restartnodehandler.h"

RestartNodeHandler::RestartNodeHandler(std::string nodename)
{
  name = nodename;
}

/**
 * Restarts the node by killing it. The launchfile should be on respawn=true, therefore the node gets restarted.
 */
void RestartNodeHandler::checkError(monitoring_msgs::Error msg) {
  char cmd[80];
  sprintf(cmd, "killall %s", name.c_str());
  system(cmd);
}
