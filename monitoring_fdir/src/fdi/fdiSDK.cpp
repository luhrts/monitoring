/*
 * FdiSDK.cpp
 *
 *  Created on: Dec 4, 2017
 *      Author: matthias
 *
 * This SDK allows you to build your own fault detection and identification.
 * You need to register the validators you like. there are some standard ones like max, min.
 * But you can write your own for complex behaiviour.
 * Inheried the ConfigInterface for new Validators
 */

#include "monitoring_fdir/fdi/fdiSDK.h"

FdiSDK::FdiSDK(ros::NodeHandle& n) {
	sub = n.subscribe("/monitoring", 10000, &FdiSDK::monitorCallback, this);
  pub = n.advertise<monitoring_msgs::Error>("/monitoring/errors", 10000);
}

FdiSDK::~FdiSDK() {
	// TODO Auto-generated destructor stub
}

/**
 * this callback will automaticly buffer the messages.
 * Which will be handled in the checkforFDI function.
 */
void FdiSDK::monitorCallback(monitoring_msgs::MonitoringArray ma) {
	for (int j = 0; j < ma.info.size(); j++) {
		for (int i = 0; i < ma.info[j].values.size(); i++) {
      monitoring_msgs::KeyValue kv = ma.info[j].values[i];
			if (!(fdiConfigList.find(kv.key) == fdiConfigList.end())) {
				//get the list with objects that are registered on this message
				std::vector<ConfigInterface *> fdiObjectList =
						fdiConfigList[kv.key];

				for (int i = 0; i < fdiObjectList.size(); i++) {
					fdiObjectList[i]->check(kv);
				}
			}
		}
	}
}


/**
 * @brief FdiSDK::registerFDIObject to register validators to check for errors in the system
 * @param object
 * @param msg
 */
void FdiSDK::registerFDIObject(ConfigInterface* object, std::string msg) {
	fdiConfigList[msg].push_back(object);
}
