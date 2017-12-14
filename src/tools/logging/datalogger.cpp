/*
 * datalogger.cpp
 *
 *  Created on: Dec 14, 2017
 *      Author: matthias
 */

#include "datalogger.h"

DataLogger::DataLogger(std::string filename, ros::NodeHandle& n)
:filename(name)
{
  sub_monitoring = n.subscribe("/monitoring/all", &DataLogger::callbackMonitors, this);
  sub_errors = n.subscribe("/monitoring/errors", &DataLogger::callbackErrors, this);

}

DataLogger::~DataLogger()
{
  // TODO Auto-generated destructor stub
}



void DataLogger::writeData(){

}

void DataLogger::callbackMonitors(ros_monitoring::MonitoringInfo mi) {
  std::string loginfo;


  logBuffer.push(loginfo);
}
void DataLogger::callbackErrors(ros_monitoring::Error errors){
  std::string loginfo;


  logBuffer.push(loginfo);
}

void DataLogger::writeSystemInformation(){
  std::ofstream logfile;
  logfile.open (filename);
  while(!(logBuffer.size() ==0)) {
    logfile << logBuffer.front();
    logBuffer.pop();
  }
  logfile.close();

}
