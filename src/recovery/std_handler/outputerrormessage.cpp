/*
 * outputerrormessage.cpp
 *
 *  Created on: Dec 20, 2017
 *      Author: matthias
 */

#include "ros_monitoring/recovery/std_handler/outputerrormessage.h"

OutputErrorMessage::OutputErrorMessage() {
	// TODO Auto-generated constructor stub

}

OutputErrorMessage::~OutputErrorMessage() {
	// TODO Auto-generated destructor stub
}

void OutputErrorMessage::checkError(ros_monitoring::Error msg)
{
	ROS_ERROR("ERROR: %s  occured. Time: %d.%d \n Value = %s , Errorlevel: %f ", msg.key.c_str(), msg.header.stamp.sec, msg.header.stamp.nsec, msg.value.c_str(), msg.errorlevel);
}
