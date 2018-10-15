#include "ping.cpp"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "monitoring_core/monitor.h"


int main(int argc, char* argv[])
{
  try
  {
    ros::init(argc, argv, "ping_monitor");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    boost::asio::io_service io_service;

    std::vector<std::string> ros_machines;
    private_nh.getParam("machines", ros_machines);

    std::vector<pinger*> pingers;

    for(std::string machine : ros_machines){
      ROS_DEBUG("Pinging: %s", machine.c_str());
      pingers.push_back(new pinger(io_service, machine.c_str()));
    }

    Monitor monitor(nh, "ping_monitor");

    while (ros::ok()) {
      io_service.run_one();

      for(int i = 0; i < ros_machines.size(); ++i)
      {
        if (pingers[i]->delay == -1){
          monitor.addValue(ros_machines[i]+"/delay", "Destination Host Unreachable", "", 1.0);
        } else {
          monitor.addValue(ros_machines[i]+"/delay", pingers[i]->delay, "ms", 0.0);
        }
      }

      ros::spinOnce();
    }


  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

}
