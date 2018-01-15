/*
 * outputerrormessage.h
 *
 *  Created on: Dec 20, 2017
 *      Author: matthias
 */

#ifndef SRC_RECOVERY_STD_HANDLER_OUTPUTERRORMESSAGE_H_
#define SRC_RECOVERY_STD_HANDLER_OUTPUTERRORMESSAGE_H_

#include "../errorhandlerinterface.h"

class OutputErrorMessage: public ErrorHandlerInterface {
public:
	OutputErrorMessage();
	virtual ~OutputErrorMessage();

	void checkError(ros_monitoring::Error msg);
};

#endif /* SRC_RECOVERY_STD_HANDLER_OUTPUTERRORMESSAGE_H_ */
