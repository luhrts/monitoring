/*
 * Monitor.cpp
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#include "monitoring_core/monitor.h"
#include <unistd.h>
#include <sstream>

#define IDEBUG 0

Monitor::Monitor(){
  init("default");
  ROS_INFO("Monitor default initialized");
}


Monitor::Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing) : miIndex(0)
{
  init(monitorDescription);
  initROS(n, autoPublishing);
  ROS_INFO("Monitor %s initialized", monitorDescription.c_str());
}


Monitor::Monitor(std::string monitorDescription, bool autoPublishing) : miIndex(0)
{
  ros::NodeHandle n;
  init(monitorDescription);
  initROS(n, autoPublishing);
  ROS_INFO("Monitor %s initialized, nodehandle created", monitorDescription.c_str());
}


Monitor::~Monitor()
{
#if IDEBUG
    ROS_INFO("Deleting %s", monitor_description_.c_str());
#endif
}


void Monitor::init(std::string monitorDescription)
{
  monitor_description_ = monitorDescription;
  monitoring_msgs::MonitoringInfo mi;

  char hostname[HOST_NAME_MAX];
  int result;
  result = gethostname(hostname, HOST_NAME_MAX);
  if (result)
  {
    perror("gethostname");
    return;
  }
  host_name_ =  hostname;
  node_name_ = ros::this_node::getName();

  mi.name = host_name_+node_name_;
  mi.description = monitorDescription;
  ma.info.push_back(mi);
  miIndex = 0;
}


void Monitor::initROS(ros::NodeHandle &n, bool autoPublishing)
{
  pub = n.advertise<monitoring_msgs::MonitoringArray> ("/monitoring", 1);

  if(autoPublishing)
  {
    float frequency = 1.0;
    ros::NodeHandle private_n("~");
    if (!private_n.getParam("monitoring/frequency", frequency))
    {
      ROS_WARN("No frequency supplied for monitoring (%s/monitoring/frequency. Working with %f Hz.", node_name_.c_str(), frequency);
    }
    ROS_INFO("%f",frequency);
    timer = n.createTimer(ros::Duration(1.0/frequency), &Monitor::timerCallback, this);
  }
  ROS_INFO("Monitor initROS done");
  if (autoPublishing)
    ROS_INFO("autoPublishing is on");
}


void Monitor::timerCallback(const ros::TimerEvent& te) {
#if IDEBUG
    ROS_INFO("Monitor %s, publish to /monitoring", monitor_description_.c_str());
#endif
  publish();
}


void Monitor::addValue(std::string key, std::string value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
#if IDEBUG
    ROS_INFO("Monitor %s, add string value, key: %s, val: %s ", monitor_description_.c_str(), key.c_str(), value.c_str());
#endif
  //check if key contains whitespace
  if (key.find_first_of("\t\n ") != std::string::npos){
    ROS_WARN("key: %s, contains a illegal char (whitespace, tab, new_line)", key.c_str());
  }
  bool is_numeric = false;
  float temp;
  try
  {
    temp = stof(value);
    is_numeric = true;
  }
  catch(...)
  {
    is_numeric = false;
  }
  if(is_numeric)
  {
    addValue(key, temp, unit, errorlevel, aggregation);
    return;
  }
  
  if ((aggregation == AggregationStrategies::MIN) || (aggregation == AggregationStrategies::MAX) || (aggregation == AggregationStrategies::AVG))
  {
    ROS_WARN("The AggregationStrategy of %s requieres a numerical value. The string doesn't contain a flaot or int", key.c_str());
    ROS_WARN("Using AggregationStrategies::LAST as fall back");
    aggregation = AggregationStrategies::LAST;
  } 
  //check if the value is already beeing monitored
  bool found = false;
  for (int i = 0; i < ma.info[miIndex].values.size(); ++i)
  {
    if ((ma.info[miIndex].values[i].key == key) && (aggregation == AggregationStrategies::LAST))
    {
        ma.info[miIndex].values[i].value = value;
        ma.info[miIndex].values[i].unit = unit;
        ma.info[miIndex].values[i].errorlevel = errorlevel;
        found = true;
        break;
    }
  }
  // if the key is new, add it to the list
  if (!found)
  {
    initNewData(key, value, unit, errorlevel);
  }
}



void Monitor::addValue(std::string key, float value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
#if IDEBUG
    ROS_INFO("Monitor %s, add float value, key: %s, val: %f ", monitor_description_.c_str(), key.c_str(), value);
#endif
  bool found = false;
  for (int i = 0; i < ma.info[miIndex].values.size(); ++i)
  {
    if (ma.info[miIndex].values[i].key == key)
    {
      if((current_values_[key].unit != unit) && (aggregation == AggregationStrategies::AVG || aggregation == AggregationStrategies::MIN || aggregation == AggregationStrategies::MAX))
            ROS_WARN("Unit of %s changed! If MIN, MAX, or AVG is selected this will produce wrong calculations", key.c_str());
      if(aggregation == AggregationStrategies::LAST)
      {
        // Always Update
        current_values_[key].value = value;
        current_values_[key].errl = errorlevel;
        current_values_[key].unit = unit;
      }
      else if (aggregation == AggregationStrategies::AVG) {
          values_for_avg_[key].sum += value;
          values_for_avg_[key].num++;
          values_for_avg_[key].errl += errorlevel;
          current_values_[key].value = values_for_avg_[key].sum/values_for_avg_[key].num;
          current_values_[key].errl = values_for_avg_[key].errl/values_for_avg_[key].num;
          current_values_[key].unit = unit;
      }
      else if (aggregation == AggregationStrategies::MIN)
      {
        if (current_values_[key].value > value)
        {
          current_values_[key].value = value;
          current_values_[key].errl = errorlevel;
          current_values_[key].unit = unit;
        }
      }
      else if (aggregation == AggregationStrategies::MAX)
      {
        if (current_values_[key].value < value)
        {
          current_values_[key].value = value;
          current_values_[key].errl = errorlevel;
          current_values_[key].unit = unit;
        }
      }
      else if (aggregation == AggregationStrategies::FIRST)
      {
        // do not update the value
        continue;
      }
      else
      {
        ROS_WARN("Unknown AggregationStrategy for key: %s", key.c_str());
        ROS_WARN("Using AggregationStrategies::Last as fall back");
        current_values_[key].value = value;
        current_values_[key].errl = errorlevel;
      }
      found = true;
      break;
    }
  }
  // if the key is new, add it to the list
  if (!found){
    initNewNumericalData(key, value, unit, errorlevel);
  }
}


void Monitor::addValue(std::string key, int value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
  addValue(key, float(value), unit, errorlevel, aggregation);
}

void Monitor::addValue(std::string key, long unsigned int value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
  addValue(key, float(value), unit, errorlevel, aggregation);
}

void Monitor::addValue(std::string key, unsigned int value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
  addValue(key, float(value), unit, errorlevel, aggregation);
}

void Monitor::addValue(std::string key, long int value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
  addValue(key, float(value), unit, errorlevel, aggregation);
}

void Monitor::addValue(std::string key, double value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
  addValue(key, float(value), unit, errorlevel, aggregation);
}


std::string Monitor::floatToString(float value)
{
    std::ostringstream temp;
    temp << value;
    std::string stringvalue(temp.str());
    //char stringvalue[1000];
    //sprintf(stringvalue, "%f", value);
    return stringvalue;
}


void Monitor::initNewData(std::string key, std::string value, std::string unit, float errorlevel)
{
    monitoring_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    kv.unit = unit;
    kv.errorlevel = errorlevel;
    ma.info[miIndex].values.push_back(kv);
}


void Monitor::initNewNumericalData(std::string key, float value, std::string unit, float errorlevel)
{
    values_for_avg_[key] = Sum();
    values_for_avg_[key].sum = value;
    values_for_avg_[key].num = 1;
    values_for_avg_[key].errl = errorlevel;
    current_values_[key] = Value();
    current_values_[key].value = value;
    current_values_[key].errl = errorlevel;
    current_values_[key].unit = unit;
    initNewData(key, floatToString(value), unit, errorlevel);
}



void Monitor::publish()
{
  ma.header.stamp = ros::Time::now();
  writeDataToMsg();
  pub.publish(ma); // achtung ist leer ??!!!??
  resetMsg();
}


void Monitor::resetMsg()
{
  monitoring_msgs::MonitoringInfo mi;
  mi.header.stamp = ros::Time::now();
  mi.name = host_name_+node_name_;
  mi.description = monitor_description_;
  ma.info.clear();
  ma.info.push_back(mi);
  current_values_.clear();
  values_for_avg_.clear();
  miIndex = 0;
}


void Monitor::writeDataToMsg()
{
  for (int i = 0; i < ma.info[miIndex].values.size(); ++i)
  {
    std::string key = ma.info[miIndex].values[i].key;
    auto search = current_values_.find(key);
    if (search != current_values_.end())
    {
        ma.info[miIndex].values[i].value = floatToString(search->second.value);
        ma.info[miIndex].values[i].errorlevel = search->second.errl;
    } 
  }
}


/*
---------------- Python-Interface-Functions -------------------
*/


void Monitor::addStringValue(char* key, char* value, char* unit, float errl, int aggregation) // for python interface
{
    AggregationStrategies agg;
    switch (aggregation){
        case 0:
            agg = AggregationStrategies::LAST;
            break;
        case 1:
            agg = AggregationStrategies::FIRST;
            break;
        case 2:
            agg = AggregationStrategies::MIN;
            break;
        case 3:
            agg = AggregationStrategies::MAX;
            break;
        case 4:
            agg = AggregationStrategies::AVG;
            break;
        default:
            agg = AggregationStrategies::LAST;
            break;
        }
#if IDEBUG
    ROS_INFO("Add string value K / V / U / E: %s, %s, %s, %f", key, value, unit, errl);
#endif
    addValue(key, value, unit, errl, agg);
}


void Monitor::addNumericValue(char* key, float value, char* unit, float errl, unsigned int aggregation) // for python interface
{
    AggregationStrategies agg;
    switch (aggregation){
        case 0:
            agg = AggregationStrategies::LAST;
            break;
        case 1:
            agg = AggregationStrategies::FIRST;
            break;
        case 2:
            agg = AggregationStrategies::MIN;
            break;
        case 3:
            agg = AggregationStrategies::MAX;
            break;
        case 4:
            agg = AggregationStrategies::AVG;
            break;
        default:
            agg = AggregationStrategies::LAST;
            break;
        }
#if IDEBUG
    ROS_INFO("Add numeric value K / V / U / E: %s, %f, %s, %f", key, value, unit, errl);
#endif
    addValue(key, value, unit, errl, agg);
}

Monitor::Monitor(int argc, char** argv, char* description, bool autoPublishing)
{
  ros::init(argc, argv, std::string(description), ros::init_options::AnonymousName);
  ROS_INFO("Starting Monitor from Python-Interface Name: %s", description);
  ros::NodeHandle n;
  init(std::string(description));
  initROS(n, autoPublishing);
}

extern "C"{
    Monitor* New_Monitor(int argc, char** argv, char* monitorDescription, bool autoPublishing)
        { return new Monitor(argc, argv, monitorDescription, autoPublishing); }

    void add_string(Monitor* monitor, char* key, char* value, char* unit, float errorlevel, unsigned int aggregation)
        { monitor->addStringValue( key, value, unit, errorlevel, aggregation); }

    void add_numeric(Monitor* monitor, char* key, float value, char* unit, float errorlevel, unsigned int aggregation)
        { monitor->addNumericValue( key, value, unit, errorlevel, aggregation); }

    void publish_and_reset(Monitor* monitor)
        { monitor->publish(); }

    void delete_monitor(Monitor* monitor)
        { printf("CPP_PY-Interface Deleting Monitor-Pointer\n"); delete monitor;}
}











