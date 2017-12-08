/*
 * cpumonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#include "cpumonitor.h"

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
void CpuMonitor::publishProcessCpuUsage(ros::Publisher pub, ros_monitoring::MonitoringInfo& mi)
{
  FILE *in;
  char buff[512];
  if (!(in = popen("ps aux", "r")))
  {
    ROS_ERROR("Could not execute ps aux");

  }
  char user[8], stat[128], command[256];
  int pid, vsz, rss, tty, starth, startm, timem, times;
  float pcpu, pmem;
  ros_monitoring::Processes list;
  int i = 0;
  while (fgets(buff, sizeof(buff), in) != NULL)
  {
    sscanf(buff, "%s %d %f %f %d %d %s %s %d:%d %d:%d %s", &user, &pid, &pcpu, &pmem, &vsz, &rss, &tty, &stat, &starth,
           &startm, &timem, &times, &command);
    //ROS_INFO("Cpu %f, %s", pcpu, command);
    ros_monitoring::Process newProc;
    newProc.name = command;
    newProc.pCpu = pcpu;
    newProc.pRam = pmem;
    newProc.pid = pid;
    newProc.stat = stat;
    list.processes.push_back(newProc);
    i++;
  }
  pclose(in);
  pub.publish(list);
}

//--------------------------------------------------------------

/*
 *	reads the tempreature from a file and transforms it to °C
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
//	ros::Publisher avg_pub, temp_pub, perc_pub, percpercore_pub, proc_pub;
  ros::Publisher monitor_pub = n.advertise<ros_monitoring::MonitoringInfo>("/monitoring/all", 1);

  int numCPU = sysconf(_SC_NPROCESSORS_ONLN);

  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
  }
  bool bAvarage = false;
  if (n.getParam("avarage", bAvarage))
  {
//		if (bAvarage) {
//			avg_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/avg", 1);
//		}
  }

  bool bPercent = false;
  if (n.getParam("percent", bPercent))
  {
//		if (bPercent) {
//			perc_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/percent",
//					1);
//		}
  }
  bool bPercentPerCore = false;
  if (n.getParam("percentPerCore", bPercentPerCore))
  {
//		if (bPercentPerCore) {
//			percpercore_pub = n.advertise<std_msgs::Float32MultiArray>("/monitoring/cpu/percentpercore",
//					1);
//		}
  }

  bool bProcesses = false;
  if (n.getParam("processes", bProcesses))
  {
//		if (bProcesses) {
//			proc_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/cpu/allProcesses", 1);
//		}
  }
  bool bTemp = false;
  if (n.getParam("temperature", bTemp))
  {
//		if (bTemp) {
//			temp_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/temp",
//					1);
//		}
  }

  ros::Rate loop_rate(freq);
  CpuMonitor cpum;

  //inti benchmark things
  ROS_RT_Benchmark benchmark;
  benchmark.init();
  ROS_RT_MeasurementDuration* measurement_percent = benchmark.createDurationMeasurement("percent");
  ROS_RT_MeasurementDuration* measurement_loadAvg = benchmark.createDurationMeasurement("load_avg");
  ROS_RT_MeasurementDuration* measurement_processcpuusage = benchmark.createDurationMeasurement("process_cpu_usage");
  ROS_RT_MeasurementDuration* measurement_temp = benchmark.createDurationMeasurement("cpu_temp");
  ROS_RT_MeasurementDuration* measurement_perCore = benchmark.createDurationMeasurement("load_per_core");

  char value[50];

  //starting looping over the options and publishing at the end
  while (ros::ok())
  {
    ros_monitoring::MonitoringInfo mi;
    mi.name = ros::this_node::getName();
    mi.description = "A CPU-Monitor";
    fillMachineInfo(mi);
    if (bPercent)
    {
      measurement_percent->start();
      float cpuload = cpum.getCurrentCpuLoad();
      measurement_percent->stop();

      ros_monitoring::KeyValue kv;
      kv.key = "overall cpu load";
      sprintf(value, "%f", cpuload);
      kv.value = value;
      mi.values.push_back(kv);
    }
    if (bAvarage)
    {
      measurement_loadAvg->start();
      float cpuavg = cpum.getLoadAvg();
      measurement_loadAvg->stop();

      ros_monitoring::KeyValue kv;
      kv.key = "cpu load avg";
      sprintf(value, "%f", cpuavg);
      kv.value = value;
      mi.values.push_back(kv);
    }
    if (bProcesses)
    {
      measurement_processcpuusage->start();
      //cpum.publishProcessCpuUsage(proc_pub, mi);
      measurement_processcpuusage->stop();
    }
    if (bTemp)
    {
      measurement_temp->start();
      float temp = cpum.getCPUTemp();
      measurement_temp->stop();

      ros_monitoring::KeyValue kv;
      kv.key = "CPU Temperatur";
      sprintf(value, "%f", temp);
      kv.value = value;
      mi.values.push_back(kv);
    }
    if (bPercentPerCore)
    {
//			std_msgs::Float32MultiArray fma;

      for (int i = 0; i < numCPU; i++)
      {
        measurement_perCore->start();
        double pCore = cpum.getCPUCoreLoad(i);
        measurement_perCore->stop();
//				fma.data.push_back(pCore);

        ros_monitoring::KeyValue kv;
        sprintf(value, "%f", pCore);
        kv.value = value;
        sprintf(value, "percentage load CoreNo: %d", i);
        kv.key = value;
        mi.values.push_back(kv);

      }
//			percpercore_pub.publish(fma);

    }

    monitor_pub.publish(mi);

//		benchmark.printProgress(true);
    loop_rate.sleep();
  }

  benchmark.logData();

}
