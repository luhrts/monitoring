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
  /**
   * @brief readWifiStrength uses the tool iwconfig to grep the signal strength
   * @param wifiInterfaceName name of the interface to control
   * @return returns the value in dBm
   */
  float readWifiStrength(std::string wifiInterfaceName);

  Monitor *monitor;
  std::string wifi_interface_name;
};

#endif // WIFISTRENGTHMONITOR_H
