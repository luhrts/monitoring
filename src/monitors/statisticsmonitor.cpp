#include "ros_monitoring/monitors/statisticsmonitor.h"

StatisticMonitor::StatisticMonitor(ros::NodeHandle &n) {
  loadConfig(n);
  stats_sub = n.subscribe("/statistics", 1000, &StatisticMonitor::statisticsCallback, this);
  msg = new MonitorMsg (n, ros::this_node::getName(), "Statistics for ROS Topics" );

  ros::Rate loop_rate(freq);
  while(ros::ok()) {
    msg->resetMsg();

    for(int i=0;i<statisticData.size(); i++) {
      msg->addNewInfoTree(statisticData[i].topic, "statistic info over the topic");
      msg->addValue("frequence", statisticData[i].frequence, "Hz", 0);
      std::string pubs, subs;
      for(int j=0;j<statisticData[i].pubs.size();j++) {
        pubs +=statisticData[i].pubs[j];
        pubs += ", ";
      }
      for(int j=0;j<statisticData[i].subs.size();j++) {
        subs +=statisticData[i].subs[j];
        subs += ", ";
      }
      msg->addValue("subscriber", subs, "", 0);
      msg->addValue("publisher", pubs, "", 0);

    }
    msg->publish();
    std::vector<StatisticsInfo> newSD;
    statisticData = newSD;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

StatisticMonitor::~StatisticMonitor() {
  delete msg;
}

void StatisticMonitor::loadConfig(ros::NodeHandle &n) {
  freq = 0.2;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }
}

void StatisticMonitor::statisticsCallback(rosgraph_msgs::TopicStatistics stats) {
  StatisticsInfo si;
  std::string topic(stats.topic);
  // ROS_INFO("mean: %f", stats.period_mean.toSec());
  for(int i=0; i<statisticData.size(); i++) {
    if(statisticData[i].topic == topic) {
      si = statisticData[i];
    }
  }
  si.topic = topic;
  si.pubs.push_back(stats.node_pub);
  si.subs.push_back(stats.node_sub);
  si.frequence = 1/stats.period_mean.toSec(); //TODO sum or mean?!?
  // ROS_INFO("Frequence: %f", si.frequence);

  statisticData.push_back(si);
}




int main(int argc, char *argv[]) {
  ros::init(argc, argv, "statistic_monitor");
  ros::NodeHandle n("~");

  StatisticMonitor sm(n);
  return 0;
}
