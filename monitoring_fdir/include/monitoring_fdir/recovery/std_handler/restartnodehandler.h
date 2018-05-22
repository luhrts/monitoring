#ifndef RESTARTNODEHANDLER_H
#define RESTARTNODEHANDLER_H

#include "../errorhandlerinterface.h"
#include <stdlib.h>

class RestartNodeHandler : public ErrorHandlerInterface
{
public:
  RestartNodeHandler();

  void checkError(monitoring_msgs::Error msg);

private:

};

#endif // RESTARTNODEHANDLER_H
