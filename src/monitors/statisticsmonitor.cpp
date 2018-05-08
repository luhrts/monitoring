#include "ros_monitoring/monitors/statisticsmonitor.h"

StatisticMonitor::StatisticMonitor(ros::NodeHandle &n) {
  n.setParam("/enable_statistics", true);
  loadConfig(n);
  stats_sub = n.subscribe("/statistics", 1000, &StatisticMonitor::statisticsCallback, this);
  msg = new MonitorMsg (n, ros::this_node::getName(), "Statistics for ROS Topics" );

  ros::Rate loop_rate(freq);
  while(ros::ok()) {
    msg->resetMsg();
    ROS_INFO("Statistics check");
    for(int i=0;i<topicRequirements.size(); i++) {
      TopicRequirement tr = topicRequirements[i];
      msg->addNewInfoTree(tr.topic, "from " + tr.source + " to " + tr.destination);
      //Find corresponding statisticdata from list
      StatisticsInfo si;
      bool siFound = false;
      for(StatisticsInfo stin:statisticData) {
        if(stin.topic==tr.topic && stin.sub==tr.destination && stin.pub==tr.source) {
          siFound = true;
          si = stin;
          break;
        }
      }
      msg->addValue("Sub: ", tr.destination, "", 0.0);
      msg->addValue("Pub: ", tr.source, "", 0.0);
      if(!siFound) {
        msg->addValue("Topic Missing", 0.0, "", 1.0);
        continue;
      }

      if(tr.frequency-tr.dFrequency <si.frequency && si.frequency < tr.frequency+tr.dFrequency){
        msg->addValue("frequency", si.frequency, "Hz", 0);
      } else {
        msg->addValue("frequency", si.frequency, "Hz", tr.errorlevel);
      }

      if(tr.size-tr.dSize <si.size && si.size < tr.size+tr.dSize){
        msg->addValue("frequency", si.size, "Byte", 0);
      } else {
        msg->addValue("frequency", si.size, "Byte", tr.errorlevel);
      }

    /*  if(tr.type!=si.type) {
        msg->addValue("type error", si.type, "", tr.errorlevel);
      } */


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
  freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }
  std::vector<std::string> topics;
  if(n.getParam("topics", topics)) {
    for(std::string name: topics) {
      TopicRequirement tr;

      if(!n.getParam(name + "/topic", tr.topic)) {
        ROS_ERROR("%s Statistics: No topic supplied.", name.c_str());
      }
      if(!n.getParam(name + "/source", tr.source)) {
        ROS_ERROR("%s Statistics: No source supplied.", name.c_str());
      }
      if(!n.getParam(name + "/source", tr.source)) {
        ROS_ERROR("%s Statistics: No source supplied.", name.c_str());
      }
      if(!n.getParam(name + "/destination", tr.destination)) {
        ROS_ERROR("%s Statistics: No destination supplied.", name.c_str());
      }
      if(!n.getParam(name + "/frequency", tr.frequency)) {
        ROS_ERROR("%s Statistics: No frequency supplied.", name.c_str());
      }
      if(!n.getParam(name + "/size", tr.size)) {
        ROS_ERROR("%s Statistics: No size supplied.", name.c_str());
      }
      if(!n.getParam(name + "/dFrequency", tr.dFrequency)) {
        ROS_ERROR("%s Statistics: No dFrequency supplied.", name.c_str());
      }
      if(!n.getParam(name + "/dSize", tr.dSize)) {
        ROS_ERROR("%s Statistics: No dSize supplied.", name.c_str());
      }
      if(!n.getParam(name + "/type", tr.type)) {
        ROS_ERROR("%s Statistics: No type supplied.", name.c_str());
      }
      if(!n.getParam(name + "/errorlevel", tr.errorlevel)) {
        ROS_ERROR("%s Statistics: No errorlevel supplied.", name.c_str());
      }
      topicRequirements.push_back(tr);
    }
  }
}

void StatisticMonitor::statisticsCallback(rosgraph_msgs::TopicStatistics stats) {
  // ROS_INFO("Statistics incoming");
  StatisticsInfo si;
  std::string topic(stats.topic);
  std::string sub(stats.node_sub);
  std::string pub(stats.node_pub);
  // ROS_INFO("Topic: %s, Pub: %s, Sub: %s",topic.c_str(), pub.c_str(),sub.c_str());
  // ROS_INFO("mean: %f", stats.period_mean.toSec());
  bool siFound = false;
  int siIndex;
  for(int i=0; i<statisticData.size(); i++) {
    if(statisticData[i].topic == topic && statisticData[i].pub == pub && statisticData[i].sub == sub) {
      si = statisticData[i];
      siFound = true;
      siIndex = i;
      break;
    }
  }

  if(!siFound)  {
    si.topic = topic;
    si.pub = pub;
    si.sub = sub;
  }
  ros::Duration difference = stats.window_stop - stats.window_start;
  double frequency = 1/stats.period_mean.toSec();
  double frequency1 = stats.delivered_msgs/difference.toSec();
  // ROS_INFO("Frequence: %f", si.frequence);
  // ROS_INFO("Frequence1: %f", frequence1);
  si.frequency =  frequency1;//TODO sum or mean?!?

  si.size = 0; //TODO

  /// si.type =  TODO

  statisticData.push_back(si);
}




int main(int argc, char *argv[]) {
  ros::init(argc, argv, "statistic_monitor");
  ros::NodeHandle n("~");

  StatisticMonitor sm(n);
  return 0;
}
