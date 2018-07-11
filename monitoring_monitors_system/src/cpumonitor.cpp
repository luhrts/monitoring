/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018  University of Hannover
 *                     Institute for Systems Engineering - RTS
 *                     Prof. Dr.-Ing. Bernardo Wagner
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name University of Hannover nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
  getloadavg(loadavg, 3); //works/updates around every 5 seconds
  return loadavg[0];
}

//-------------------------------------------------------

/**
 * getting all starting values of the file /proc/stat at first because they are incrementel, so we need them for the delta. They are
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
    fscanf(fname, "%s", name);
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

  int old_n = n;

  char buffer[256];
  for (int i = 0; i <= n; i++)
  {
    fgets(buffer, 256, file);
  }
  fscanf(file, "cpu%d %llu %llu %llu %llu", &n, &totalUser, &totalUserLow, &totalSys, &totalIdle);
  fclose(file);

  if (totalUser < lastTotalUser[n+1] || totalUserLow < lastTotalUserLow[n+1] || totalSys < lastTotalSys[n+1]
      || totalIdle < lastTotalIdle[n+1])
  {
    //Overflow detection. Just skip this value.
    percent = -1.0;
  }
  else
  {
    //because the value is incrementel, you need to calculate the deltas
    total = (totalUser - lastTotalUser[n+1]) + (totalUserLow - lastTotalUserLow[n+1]) + (totalSys - lastTotalSys[n+1]);	//if total is 0 a nan value is possible TODO!!!
    percent = total;
    total += (totalIdle - lastTotalIdle[n+1]);
    percent /= total;
    percent *= 100;

  }

  lastTotalUser[n+1] = totalUser;
  lastTotalUserLow[n+1] = totalUserLow;
  lastTotalSys[n+1] = totalSys;
  lastTotalIdle[n+1] = totalIdle;
  return percent;

}

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

  double freq;
  n.param<double>("frequency", freq, 1.0);

  bool bAvarage;
  n.param<bool>("avarage", bAvarage, true);

  bool bPercent ;
  n.param<bool>("percent", bPercent, true);

  bool bPercentPerCore;
  n.param<bool>("percentPerCore", bPercentPerCore, true);

  bool bTemp;
  n.param<bool>("temperature", bTemp, true);

  ros::Rate loop_rate(freq);
  CpuMonitor cpum;

  Monitor msg(n, "A CPU-Monitor");

  //starting looping over the options and publishing at the end
  while (ros::ok())
  {

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
    ros::spinOnce();
    loop_rate.sleep();
  }
}
