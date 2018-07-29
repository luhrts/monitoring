#include <ros/ros.h>
#include <monitoring_core/monitor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "performance_monitor_test_node");
  ros::NodeHandle nh;

  Monitor monitor(nh, "performance_monitor_test_node");

  ros::Rate loop_rate(2000);
  int c = 0;
  while (ros::ok()){
    monitor.addValue("count", c, "SIUnits", 0.0);
    c++;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
