/*
 * guiconcatenation.h
 *
 *  Created on: Jan 5, 2018
 *      Author: matthias
 */

#ifndef SRC_GUICONCATENATION_H_
#define SRC_GUICONCATENATION_H_

#include "ros/ros.h"
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/Error.h"
#include "monitoring_msgs/Gui.h"

class GuiConcatenation {
public:
	GuiConcatenation(ros::NodeHandle& n,int mode);
	virtual ~GuiConcatenation();

  /**
   * @brief getMsg function to get the msg for publishing
   * @return msg type for aggregated monitors and error msgs
   */
  monitoring_msgs::Gui getMsg();
private:
  /**
   * @brief monitor_cb callback for monitoring msgs
   * @param ma
   */
  struct info_Startegy
 {
  std::string name;
  std::string max;
  std::string min;
  std::string first;
  std::string last;
  
 };

  std::map<std::string,info_Startegy>  IS;

  void monitor_cb(monitoring_msgs::MonitoringArray ma);
  /**
   * @brief error_cb callback for error msgs
   * @param er
   */
  int MODE;
  void error_cb(monitoring_msgs::Error er);
  ros::Subscriber monitor_sub, error_sub; ///< subscribers for data collection
  monitoring_msgs::Gui msg;
};

#endif /* SRC_GUICONCATENATION_H_ */
