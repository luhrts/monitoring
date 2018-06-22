#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <monitoring_core/monitor.h>


std::string topic_name_;
std::string monitoring_key_;
std::string si_unit_;
double rate_;
Monitor* monitor_;


void cb_value(std_msgs::Float32 value) {
    monitor_->addValue(monitoring_key_, value.data, si_unit_, 0.0);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "topic_value_monitor");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("topic_name", topic_name_, "ERROR");
  private_nh.param<std::string>("monitoring_key", monitoring_key_, "ERROR");
  private_nh.param<std::string>("si_unit", si_unit_, "ERROR");
  private_nh.param<double>("rate", rate_, 1.0);


  monitor_ = new Monitor(nh, "topic_value_monitor");

  ros::Subscriber sub_value = nh.subscribe(topic_name_, 1, cb_value);

  ros::spin();

}
