#include "monitoring_monitors_ros/statisticsmonitor.h"

StatisticMonitor::StatisticMonitor(ros::NodeHandle &n) {
  n.setParam("/enable_statistics", true);
  loadConfig(n);
  stats_sub = n.subscribe("/statistics", 1000, &StatisticMonitor::statisticsCallback, this);
  msg = new Monitor (n, "Statistics for ROS Topics" );

  ros::Rate loop_rate(freq);
  while(ros::ok()) {

    // ROS_INFO("Statistics check");
    compareStatisticDataWithRequirements();

//    std::vector<StatisticsInfo> newSD;
//    statisticData = newSD;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

StatisticMonitor::~StatisticMonitor() {
  delete msg;
}

void StatisticMonitor::deleteOldMessages() {
  ros::Time now =ros::Time::now();
  for(int i=statisticData.size()-1;i>=0; i--) {
    ros::Duration oldness = now-statisticData[i].time;
    if(oldness.toSec() > timeTilDeletingOldMessages) {
      statisticData.erase(statisticData.begin()+i);
    }
  }
}

void StatisticMonitor::compareStatisticDataWithRequirements() {
//  ROS_INFO("compare");
  deleteOldMessages();
  for(int i=0;i<topicRequirements.size(); i++) {
    TopicRequirement tr = topicRequirements[i];
 //   msg->addNewInfoTree(tr.topic, "from " + tr.source + " to " + tr.destination);   //TODO: Find new way to handle
    //ROS_INFO("Topic: %s", tr.topic.c_str());
    //Find corresponding statisticdata from list
    StatisticsInfo si;
//    ROS_INFO("statisticData size: %d", statisticData.size());
    bool siFound = false;
    for(StatisticsInfo stin:statisticData) { //searching for coresponding recorded statistic data
//      ROS_INFO("COMPARE: %s, %s - %s, %s - %s, %s", stin.topic.c_str(), tr.topic.c_str(), stin.sub.c_str(), tr.destination.c_str(), stin.pub.c_str(), tr.source.c_str());
      if(stin.topic==tr.topic && stin.sub==tr.destination && stin.pub==tr.source) {
        siFound = true;
        si = stin;
        break;
      }
    }
    msg->addValue(tr.topic+"/Sub: ", tr.destination, "", 0.0);
    msg->addValue(tr.topic+"/Pub: ", tr.source, "", 0.0);
    if(!siFound) {//testing if topic is available
      msg->addValue(tr.topic+ "/TopicMissing", 0.0, "", 1.0);
      ROS_WARN("Topic missing: %s, source: %s , dest: %s",tr.topic.c_str(),tr.source.c_str(), tr.destination.c_str());
      continue;
    }
//    ROS_ERROR("Topic richtig");
    //checking frequency
    if(tr.frequency-tr.dFrequency <si.frequency && si.frequency < tr.frequency+tr.dFrequency){
      msg->addValue(tr.topic+ "/frequency", si.frequency, "Hz", 0);
    } else {
      ROS_WARN("%s wrong freuqency: %f",tr.topic.c_str(), si.frequency);
      if(tr.frequency != -1) { //if frequency is -1, it should not be checked
        msg->addValue(tr.topic+ "/frequency", si.frequency, "Hz", tr.errorlevel);
      }
    }
//    ROS_INFO("Frequency checked");

    if(tr.size-tr.dSize <si.size && si.size < tr.size+tr.dSize){
      msg->addValue(tr.topic+ "/size", si.size, "Byte", 0);
    } else {
      if(tr.size != -1) { //if size is -1, it should not be checked
        msg->addValue(tr.topic+ "/size", si.size, "Byte", tr.errorlevel);
      }
    }
//    ROS_INFO("Size checked");


  /*  if(tr.type!=si.type) { //type is not included in statistics
      msg->addValue("type error", si.type, "", tr.errorlevel);
    } */


  }
}

void StatisticMonitor::loadConfig(ros::NodeHandle &n) {
  freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }

  timeTilDeletingOldMessages= 3.0;
  if (!n.getParam("timeTilDelete", timeTilDeletingOldMessages))
  {
    ROS_WARN("No time til delete supplied. Working with %f sec.", timeTilDeletingOldMessages);
  }


  std::vector<std::string> topics;
  if(n.getParam("topics", topics)) {
    for(std::string name: topics) {
      TopicRequirement tr;

      if(!n.getParam(name + "/topic", tr.topic)) {
        ROS_ERROR("%s Statistics: No topic supplied.", name.c_str());
      } else {
        if(tr.topic.front() != '/') {
          tr.topic.insert(0, "/");
        }
      }

      if(!n.getParam(name + "/source", tr.source)) {
        ROS_ERROR("%s Statistics: No source supplied.", name.c_str());
      }else {
        if(tr.source.front() != '/') {
          tr.source.insert(0, "/");
        }
      }
      if(!n.getParam(name + "/destination", tr.destination)) {
        ROS_ERROR("%s Statistics: No destination supplied.", name.c_str());
      }else {
        if(tr.destination.front() != '/') {
          tr.destination.insert(0, "/");
        }
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
//  ROS_INFO("Statistics incoming");
  StatisticsInfo si;
  std::string topic(stats.topic);
  std::string sub(stats.node_sub);
  std::string pub(stats.node_pub);
  // ROS_INFO("Topic: %s, Pub: %s, Sub: %s",topic.c_str(), pub.c_str(),sub.c_str());
  // ROS_INFO("mean: %f", stats.period_mean.toSec());
  bool siFound = false;
  int siIndex;
//  ROS_INFO("check if already inserted");
  for(int i=0; i<statisticData.size(); i++) {
    if(statisticData[i].topic == topic && statisticData[i].pub == pub && statisticData[i].sub == sub) {
      si = statisticData[i];
      siFound = true;
      siIndex = i;
      break;
    }
  }
//  ROS_INFO("create new one?");
  if(!siFound)  {
    si.topic = topic;
    si.pub = pub;
    si.sub = sub;
  }
//  ROS_INFO("calculate");
  ros::Duration difference = stats.window_stop - stats.window_start;
  double frequency = 1/stats.period_mean.toSec();
  double frequency1 = stats.delivered_msgs/difference.toSec();
  // ROS_INFO("Frequence: %f", si.frequence);
  // ROS_INFO("Frequence1: %f", frequence1);
//  ROS_INFO("set values");
  si.frequency =  frequency1;//TODO sum or mean?!?
//  ROS_INFO("delmsgs: %f, traffic: %f", stats.delivered_msgs, stats.traffic);
  if(stats.traffic == 0) {
    si.size = 0;
  } else {
    si.size = stats.delivered_msgs/stats.traffic; //TODO
  }
  si.time = ros::Time::now();

  /// si.type =  TODO
//  ROS_INFO("pushback");
  statisticData.push_back(si);
}




int main(int argc, char *argv[]) {
  ros::init(argc, argv, "statistic_monitor");
  ros::NodeHandle n("~");

  StatisticMonitor sm(n);
  return 0;
}
