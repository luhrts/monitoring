#ifndef WIFISTRENGTHMONITOR_H
#define WIFISTRENGTHMONITOR_H

#include "ros/ros.h"
#include "monitoring_core/monitor.h"

/**
 * @brief The WifiStrengthMonitor class monitors the wifi signal strength over the tool iwconfig
 */
class WifiStrengthMonitor
{
public:
  WifiStrengthMonitor(ros::NodeHandle &n);
  ~WifiStrengthMonitor();

private:
  Monitor *monitor;
  std::string wifi_interface_name;
};

#endif // WIFISTRENGTHMONITOR_H
