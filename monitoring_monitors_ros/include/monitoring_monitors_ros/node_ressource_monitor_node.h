/*
 *  Copyright (c) 2018, University of Hannover
 *                      Institute for Systems Engineering - RTS
 *                      Professor Bernardo Wagner
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */
#ifndef SRC_NODERESSOURCEMONITOR_H_
#define SRC_NODERESSOURCEMONITOR_H_

#include "ros/ros.h"
#include "monitoring_core/monitor.h"

//#define _GNU_SOURCE     /* To get defns of NI_MAXSERV and NI_MAXHOST */
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/if_link.h>
#include <map>
#include <dirent.h>
#include <time.h> // defines CLOCKS_PER_SEC
#include <unistd.h> // defines _SC_CLK_TCK; _SC_PAGE_SIZE; long sysconf(int name)
#include <iostream>
#include <fstream>
#include <regex>
#include <algorithm>

#include <cstdio>
#include <cstdarg>


struct PidStat{
  int               pid;
  char              comm[30];
  char              state;
  int               ppid;
  int               pgrp;
  int               session;
  int               tty_nr;
  int               tpgid;
  unsigned int      flags;
  unsigned long int minflt;
  unsigned long int cminflt;
  unsigned long int majflt;
  unsigned long int cmajflt;
  unsigned long int utime;
  unsigned long int stime;
  long int          cutime;
  long int          cstime;
  long int          priority;
  long int          nice;
  long int          num_threads;
  long int          itrealvalue;
  unsigned long long int starttime;
  unsigned long int vsize;
  long int          rss;
  unsigned long int rsslim;
  unsigned long int startcode;
  unsigned long int encode;
  unsigned long int startstack;
  unsigned long int kstkesp;
  unsigned long int kstkeip;
  unsigned long int signal;
  unsigned long int blocked;
  unsigned long int sigignore;
  unsigned long int sigcatch;
  unsigned long int wchan;
  unsigned long int nswap;
  unsigned long int cnswap;
  int               exit_sig;
  int               processor;
  unsigned int      rt_priority;
  unsigned int      policy;
  unsigned long long int delayacct_blkio_ticks;
  unsigned long int guest_time;
  long int          cguest_time;
  unsigned long int start_data;
  unsigned long int end_data;
  unsigned long int start_brk;
  unsigned long int arg_start;
  unsigned long int arg_end;
  unsigned long int env_start;
  unsigned long int env_end;
  int               exit_code;
  float             cpu_perc;
  ros::Time         last_u_time;
  unsigned long int voluntary_ctxt_switches;
  unsigned long int nonvoluntary_ctxt_switches;
  double          user_t;
  double          sys_t;
};

class NodeResMon{
    public:
        NodeResMon(ros::NodeHandle &n);
        ~NodeResMon();
        void run();
    private:
        void init();
        void updateNodeValues();
        void publishNodeInfos();
        void checkNewNodes();
        void getNodeInfos();
        void updateNodes(bool init);
        void loadConfig(ros::NodeHandle &n);
        void readargs(const char *buff, const char *fmt, ...);

        int cpuCount();

        std::string statFormatStr;
        std::string ips;
        std::string interfaces;
        std::vector<std::string> pid_s;
        std::map<std::string, PidStat> node_map;
        float checkNewNodesUpdateIntervall;
        int freq;
        int monitor_mode;
        long page_size_kb;
        long clock_ticks_per_sec;

        ros::Rate loop_rate;
        ros::Time nodeUpdateTime;
        AggregationStrategies aggregation;
        Monitor *msg;
};

#endif
