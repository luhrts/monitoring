#include <ros/ros.h>
#include <monitoring_core/monitor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Gao_monitor");
  ros::NodeHandle nh;

  Monitor monitor_1(nh, "Gao_monitor_int_example");
  Monitor monitor_2(nh, "Gao_monitor_string_example");


  ros::Rate loop_rate(10);
  double E_level;
  int x = 0;
  std::string y="Hello monitor";
  while (ros::ok()){
      if(x>=20)
      {
          E_level=1.0;
        if(x>=40)
        {
            E_level=2.0;
        }
      }
      else
           E_level=0.0;

    monitor_2.addValue("count_level", x, "Number unit",E_level);
    monitor_1.addValue("char", y, "Char unit",0.0);
    x++;
    monitor_1.publish();
    monitor_2.publish();

    loop_rate.sleep();
  }


  ROS_INFO("Hello world!");
}
