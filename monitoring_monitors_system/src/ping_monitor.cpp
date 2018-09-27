#include "ping.cpp"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "monitoring_core/monitor.h"


int main(int argc, char* argv[])
{
  try
  {
    ros::init(argc, argv, "ping_monitor");

    boost::asio::io_service io_service;

    std::vector<std::string> ros_machines;
    ros_machines.push_back("google.de");
    ros_machines.push_back("130.75.137.10");

    std::vector<pinger*> pingers;

    for(std::string machine : ros_machines){
      pingers.push_back(new pinger(io_service, machine.c_str()));
    }

    ros::NodeHandle nh;
    Monitor monitor(nh, "ping_monitor");

    while (ros::ok()) {
      io_service.run_one();

      for(int i = 0; i < ros_machines.size(); ++i)
      {
        monitor.addValue(ros_machines[i]+"/delay", pingers[i]->delay, "ms", 0.0);
      }

      ros::spinOnce();
    }


  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

}
