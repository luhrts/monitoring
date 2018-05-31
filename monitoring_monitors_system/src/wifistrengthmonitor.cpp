#include "monitoring_monitors_system/wifistrengthmonitor.h"

WifiStrengthMonitor::WifiStrengthMonitor(ros::NodeHandle &n)
{


  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
  }

  if (!n.getParam("wifiInterface", wifi_interface_name))
  {
    ROS_ERROR("No interface name supplied supplied. EXIT");
    return;
  }
  ros::Rate loop_rate(freq);
  monitor = new Monitor(n, "Wifi Signal Strength Monitor");
  while (ros::ok())
  {
    monitor->addValue("Wifi Strength", readWifiStrength(wifi_interface_name), "dBm", 0);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

WifiStrengthMonitor::~WifiStrengthMonitor() {
  delete monitor;
}

float WifiStrengthMonitor::readWifiStrength(std::string wifiInterfaceName){
  char cmd[300];
  sprintf(cmd, "iwconfig %s|grep Signal|cut -d\"=\" -f3|cut -d\" \" -f1", wifiInterfaceName.c_str());
  FILE* cmd_ret = popen(cmd, "r");
  if(!cmd_ret) {
    ROS_WARN("could not read wlan signal strength!!!");
    return 0;
  }
  char line[200];
  float wifistrength = 0;
  if(fgets(line, 200, cmd_ret) != NULL) {
    wifistrength = atof(line);
    return wifistrength;
  }
  return 0;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_strength_monitor");
  ros::NodeHandle n("~");
  WifiStrengthMonitor wifimon(n);


}
