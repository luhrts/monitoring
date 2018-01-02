/*
 * youbotbatteriemonitor.h
 *
 *  Created on: Jan 2, 2018
 *      Author: matthias
 */

#ifndef SRC_MONITORS_YOUBOTBATTERIEMONITOR_H_
#define SRC_MONITORS_YOUBOTBATTERIEMONITOR_H_

class YoubotBatterieMonitor {
public:
	YoubotBatterieMonitor();
	virtual ~YoubotBatterieMonitor();

private:
	void setupSerial();
	void getVoltages();
};

#endif /* SRC_MONITORS_YOUBOTBATTERIEMONITOR_H_ */
