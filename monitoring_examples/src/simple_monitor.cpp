#include <ros/ros.h>
#include <monitoring_core/monitor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_monitor");
  ros::NodeHandle nh;

  Monitor monitor(nh, "simple_monitor_example");

  ros::Rate loop_rate(10);
  int c = 0;
  while (ros::ok()){
    monitor.addValue("count", c, "SIUnits", 0.0);
    c++;
    monitor.publish();
    loop_rate.sleep();
  }


  ROS_INFO("Hello world!");
}
