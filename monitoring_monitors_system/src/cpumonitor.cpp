/*
 * cpumonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#include "monitoring_monitors_system/cpumonitor.h"

CpuMonitor::CpuMonitor()
{
  init();
}

CpuMonitor::~CpuMonitor()
{
}

float CpuMonitor::getLoadAvg()
{
  double loadavg[3];
  std_msgs::Float32 avg;
  getloadavg(loadavg, 3); //works/updates around every 5 seconds
  return loadavg[0];
}

//-------------------------------------------------------

//Quelle: https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
/**
 * getting all starting values of the cpu at first because they are incrementel, so we need them for the delta. They are
 * catched for every core and overall cpu and saved to a vector.
 */
void CpuMonitor::init()
{

  int numCPU = sysconf(_SC_NPROCESSORS_ONLN);

  unsigned long long lastTotalUser_, lastTotalUserLow_, lastTotalSys_, lastTotalIdle_;

  FILE* file = fopen("/proc/stat", "r");
  fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser_, &lastTotalUserLow_, &lastTotalSys_, &lastTotalIdle_);//read overall cpu values

  lastTotalUser.push_back(lastTotalUser_);
  lastTotalUserLow.push_back(lastTotalUserLow_);
  lastTotalSys.push_back(lastTotalSys_);
  lastTotalIdle.push_back(lastTotalIdle_);
  char buff[256];
  int n;

  fgets(buff, sizeof(buff), file); //to jump over the first line (already done thatone)
  while (fgets(buff, sizeof(buff), file) != NULL)
  { //read for every cpu core

    sscanf(buff, "cpu%d %llu %llu %llu %llu", &n, &lastTotalUser_, &lastTotalUserLow_, &lastTotalSys_, &lastTotalIdle_);

    lastTotalUser.push_back(lastTotalUser_);
    lastTotalUserLow.push_back(lastTotalUserLow_);
    lastTotalSys.push_back(lastTotalSys_);
    lastTotalIdle.push_back(lastTotalIdle_);

    if (n == numCPU - 1)
    { //read last cpu, stop now
      break;
    }
  }
  fclose(file);

  //tempreature file init
  bool corefound = false;
  int i = -1;
  char path[80];
  while (!corefound && i < 2)
  { //TODO: was wenn kein coretemp gefunden || Probleme mit i! TODO: in den construcktor verschieben.
    i++;
    FILE* fname;
    sprintf(path, "/sys/class/hwmon/hwmon%d/name", i);
    fname = fopen(path, "r");
    char name[30];
    fscanf(fname, "%s", &name);
    fclose(fname);
    if (strcmp(name, "coretemp") == 0)
    {	//check which is coretemp

      corefound = true;
      break;
    }
  }
  temp_index = i;

}

/*
 * Reads values in jiffies and calculates the Load of the overall Cpu in percent
 */
float CpuMonitor::getCurrentCpuLoad()
{
  float percent;
  FILE* file;
  unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

  //reads values from proc file system
  file = fopen("/proc/stat", "r");
  fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow, &totalSys, &totalIdle);
  fclose(file);

//    ROS_INFO("cpu %llu %llu %llu %llu", totalUser, totalUserLow,
//            totalSys, totalIdle);

  if (totalUser < lastTotalUser[0] || totalUserLow < lastTotalUserLow[0] || totalSys < lastTotalSys[0]
      || totalIdle < lastTotalIdle[0])
  {
    //Overflow detection. Just skip this value.
    percent = -1.0;
  }
  else
  {
    //because the value is incrementel you need to get the delta values
    total = (totalUser - lastTotalUser[0]) + (totalUserLow - lastTotalUserLow[0]) + (totalSys - lastTotalSys[0]);	//if total is 0 a nan value is possible TODO!!!

    percent = total;
    total += (totalIdle - lastTotalIdle[0]);
    percent /= total;
    percent *= 100;

  }

  lastTotalUser[0] = totalUser;
  lastTotalUserLow[0] = totalUserLow;
  lastTotalSys[0] = totalSys;
  lastTotalIdle[0] = totalIdle;

  return percent;
}
/*
 * Reads values in jiffies and calculates the Load of the n-th Cpu core in percent
 */
double CpuMonitor::getCPUCoreLoad(int n)
{ //TODO need to test if this is the right cpu core load
  double percent;
  FILE* file;
  unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

  file = fopen("/proc/stat", "r");

  char buffer[256];
  for (int i = 0; i <= n; i++)
  {
    fgets(buffer, 256, file);
  }
  fscanf(file, "cpu%d %llu %llu %llu %llu", &n, &totalUser, &totalUserLow, &totalSys, &totalIdle);
  fclose(file);

  if (totalUser < lastTotalUser[n] || totalUserLow < lastTotalUserLow[n] || totalSys < lastTotalSys[n]
      || totalIdle < lastTotalIdle[n])
  {
    //Overflow detection. Just skip this value.
    percent = -1.0;
  }
  else
  {
    //because the value is incrementel, you need to get the deltas
    total = (totalUser - lastTotalUser[n]) + (totalUserLow - lastTotalUserLow[n]) + (totalSys - lastTotalSys[n]);	//if total is 0 a nan value is possible TODO!!!
    //        ROS_INFO("total: %llu", total);
    percent = total;
    total += (totalIdle - lastTotalIdle[n]);
    percent /= total;
    percent *= 100;

  }

  lastTotalUser[n] = totalUser;
  lastTotalUserLow[n] = totalUserLow;
  lastTotalSys[n] = totalSys;
  lastTotalIdle[n] = totalIdle;
  return percent;

}

//--------------------------------------------------------------

/*
 * This gets information from ps aux, parses it and publishes every process
 *
 * DOES NOT WORK ATM
 */
//void CpuMonitor::publishProcessCpuUsage(ros::Publisher pub, ros_monitoring::MonitoringInfo& mi)
//{
//  FILE *in;
//  char buff[512];
//  if (!(in = popen("ps aux", "r")))
//  {
//    ROS_ERROR("Could not execute ps aux");

//  }
//  char user[8], stat[128], command[256];
//  int pid, vsz, rss, tty, starth, startm, timem, times;
//  float pcpu, pmem;
//  ros_monitoring::Processes list;
//  int i = 0;
//  while (fgets(buff, sizeof(buff), in) != NULL)
//  {
//    sscanf(buff, "%s %d %f %f %d %d %s %s %d:%d %d:%d %s", &user, &pid, &pcpu, &pmem, &vsz, &rss, &tty, &stat, &starth,
//           &startm, &timem, &times, &command);
//    //ROS_INFO("Cpu %f, %s", pcpu, command);
//    ros_monitoring::Process newProc;
//    newProc.name = command;
//    newProc.pCpu = pcpu;
//    newProc.pRam = pmem;
//    newProc.pid = pid;
//    newProc.stat = stat;
//    list.processes.push_back(newProc);
//    i++;
//  }
//  pclose(in);
//  pub.publish(list);
//}

//--------------------------------------------------------------

/*
 *	reads the temperature from a file and transforms it to °C
 */

float CpuMonitor::getCPUTemp()
{

  char path[80];
  FILE* file;
  sprintf(path, "/sys/class/hwmon/hwmon%d/temp1_input", temp_index);
  file = fopen(path, "r");
  int input = 0;
  fscanf(file, "%d", &input);
  fclose(file);
  float temp = input / 1000; //in °C

  return temp;
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cpu_monitor");
  ros::NodeHandle n("~");

  int numCPU = sysconf(_SC_NPROCESSORS_ONLN);


  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
  }
  bool bAvarage = false;
  if (n.getParam("avarage", bAvarage))
  {
  }

  bool bPercent = false;
  if (n.getParam("percent", bPercent))
  {
  }
  bool bPercentPerCore = false;
  if (n.getParam("percentPerCore", bPercentPerCore))
  {
  }

  bool bProcesses = false;
  if (n.getParam("processes", bProcesses))
  {
  }
  bool bTemp = false;
  if (n.getParam("temperature", bTemp))
  {
  }

  ros::Rate loop_rate(freq);
  CpuMonitor cpum;

  //inti benchmark things

  char value[50];
  Monitor msg(n, ros::this_node::getName(), "A CPU-Monitor");


  //starting looping over the options and publishing at the end
  while (ros::ok())
  {
    msg.resetMsg();
//    msg.addNewInfoTree("CPU", "CPU Data");

    if (bPercent)
    {

      float cpuload = cpum.getCurrentCpuLoad();

      msg.addValue("overall cpu load", cpuload, "%", 0);
    }
    if (bAvarage)
    {
      float cpuavg = cpum.getLoadAvg();
      msg.addValue("cpu load avg", cpuavg, "", 0);
    }
    if (bProcesses)
    {
      //cpum.publishProcessCpuUsage(proc_pub, mi);
    }
    if (bTemp)
    {
      float temp = cpum.getCPUTemp();

      msg.addValue("CPU Temperatur", temp, "C", 0);
    }
    if (bPercentPerCore)
    {
      for (int i = 0; i < numCPU; i++)
      {
        double pCore = cpum.getCPUCoreLoad(i);

        char key[40];
        sprintf(key, "percentage load CoreNo: %d", i);
        msg.addValue(key, pCore, "%", 0);

      }

    }
    msg.publish();

    loop_rate.sleep();
  }

}
