/*
 * topicmonitor.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: matthias
 *
 *      ------------------ DEPRICATED ------------------
 */

#include "topicmonitor.h"

TopicMonitor::TopicMonitor()
{
  // TODO Auto-generated constructor stub

}

TopicMonitor::~TopicMonitor()
{
  // TODO Auto-generated destructor stub
}

std::vector<std::string> TopicMonitor::getTopics()
{
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  std::vector < std::string > ret;
  for (int i = 0; i < topics.size(); i++)
  {
    ret.push_back(topics[i].name);
  }
  return ret;
}

void generateFile(char* filename)
{
  FILE* file;
  file = fopen(filename, "w");
  TopicMonitor topicMon;
  std::vector < std::string > topics = topicMon.getTopics();
  for (int i = 0; i < topics.size(); i++)
  {
    fprintf(file, "%s\n", topics[i].c_str());
  }
  fclose(file);
  ROS_INFO("Found %d Topics", topics.size());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "topic_monitor");
  ros::NodeHandle n("~");
  ros::Publisher monitor_pub = n.advertise<ros_monitoring::MonitoringInfo>("/monitoring/all", 1);

  if (argc > 1)
  {
    if (!strcmp(argv[1], "-generate"))
    {
      ROS_INFO("Fetching topics and saving to %s", argv[2]);
      generateFile(argv[2]);
    }
    else
    {
      ROS_ERROR("%s is not a valid option, try -generate $file", argv[1]);
    }
    return 0;
  }

  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }

  std::string filename = "nodes.txt";
  if (!n.getParam("nodefile", filename))
  {
    ROS_WARN("No Network Interface configured. Using %s", filename.c_str());
  }

  ros::Rate loop_rate(freq);

  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  for (int i = 0; i < topics.size(); i++)
  {

    ROS_INFO("%s: %s", topics[i].name.c_str(), topics[i].datatype.c_str());
  }

  while (ros::ok())
  {

    loop_rate.sleep();
  }

}
