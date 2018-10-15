#include "monitoring_monitors_system/wifi_monitor.h"

#include <iostream>
#include <fstream>

WifiStrengthMonitor::WifiStrengthMonitor(ros::NodeHandle &n)
{
  double freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }

  ros::Rate loop_rate(freq);
  monitor = new Monitor(n, "Wifi Signal Strength Monitor");
  while (ros::ok())
  {

    std::string line;
    std::ifstream input("/proc/net/wireless");

    //Skip first two lines since they only contain header info
    std::getline(input, line);
    std::getline(input, line);
    while(std::getline(input, line))
    {
      //      ROS_INFO("%s", line.c_str());
      std::string interface;
      char ifname[20];
      int status, noise, link, level;
      sscanf(line.c_str(), "%[^:]: %d %d . %d . %d . %*d %*d %*d %*d %*d %*d", ifname, &status, &link, &level, &noise);
      //      ROS_INFO("interface: %s, level: %d, link: %d, noise: %d", ifname, link, level, noise);

      interface = std::string(ifname);

      monitor->addValue(interface+"/signal_strength", level, "dBm", 0);
      monitor->addValue(interface+"/quality", link, "", 0);
      monitor->addValue(interface+"/noise", noise, "", 0);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

WifiStrengthMonitor::~WifiStrengthMonitor() {
  delete monitor;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_strength_monitor");
  ros::NodeHandle n("~");
  WifiStrengthMonitor wifimon(n);


}
