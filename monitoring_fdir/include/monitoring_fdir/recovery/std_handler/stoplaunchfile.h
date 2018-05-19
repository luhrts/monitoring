/*
 * stoplaunchfile.h
 *
 *  Created on: Dec 18, 2017
 *      Author: matthias
 */

#ifndef SRC_RECOVERY_STD_HANDLER_STOPLAUNCHFILE_H_
#define SRC_RECOVERY_STD_HANDLER_STOPLAUNCHFILE_H_

#include "../errorhandlerinterface.h"

class StopLaunchFile : public ErrorHandlerInterface
{
public:
  StopLaunchFile();
  virtual ~StopLaunchFile();

  void checkError(ros_monitoring::Error msg);
};

#endif /* SRC_RECOVERY_STD_HANDLER_STOPLAUNCHFILE_H_ */
