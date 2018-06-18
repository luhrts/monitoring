#ifndef RESTARTNODEHANDLER_H
#define RESTARTNODEHANDLER_H

#include "../errorhandlerinterface.h"
#include <stdlib.h>

class RestartNodeHandler : public ErrorHandlerInterface
{
public:
  RestartNodeHandler(std::string nodename);

  void checkError(monitoring_msgs::Error msg);

private:
  std::string name;
};

#endif // RESTARTNODEHANDLER_H
