/*
 * cpumonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#include "cpumonitor.h"

CpuMonitor::CpuMonitor() {
	init();

}

CpuMonitor::~CpuMonitor() {
	// TODO Auto-generated destructor stub
}

void CpuMonitor::publishLoadAvg(ros::Publisher pub) {
	double loadavg[3];
	std_msgs::Float32 avg;
	getloadavg(loadavg, 3); //works/updates around every 5 seconds
	//ROS_INFO(" %f, %f, %f", loadavg[0],loadavg[1],loadavg[2]);

	avg.data = loadavg[0];
	pub.publish(avg);
}

//-------------------------------------------------------

//Quelle: https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
void CpuMonitor::init() {
	FILE* file = fopen("/proc/stat", "r");
	fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser, &lastTotalUserLow,
			&lastTotalSys, &lastTotalIdle);
	fclose(file);
}

double CpuMonitor::getCurrentValue() {
	double percent;
	FILE* file;
	unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

	//reads values from proc file system
	file = fopen("/proc/stat", "r");
	fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow,
			&totalSys, &totalIdle);
	fclose(file);

//    ROS_INFO("cpu %llu %llu %llu %llu", totalUser, totalUserLow,
//            totalSys, totalIdle);

	if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow
			|| totalSys < lastTotalSys || totalIdle < lastTotalIdle) {
		//Overflow detection. Just skip this value.
		percent = -1.0;
	} else {
		total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow)
				+ (totalSys - lastTotalSys);//if total is 0 a nan value is possible TODO!!!
//        ROS_INFO("total: %llu", total);
		percent = total;
		total += (totalIdle - lastTotalIdle);
		percent /= total;
		percent *= 100;

	}

	lastTotalUser = totalUser;
	lastTotalUserLow = totalUserLow;
	lastTotalSys = totalSys;
	lastTotalIdle = totalIdle;

	return percent;
}

void CpuMonitor::publishCpuUsage(ros::Publisher pub) {

	double pCPU = getCurrentValue();
//	ROS_INFO("Load: %f ", pCPU);
	std_msgs::Float32 cpu_msg;
	cpu_msg.data = pCPU;
	pub.publish(cpu_msg);
}

//--------------------------------------------------------------

void CpuMonitor::publishProcessCpuUsage(ros::Publisher pub) {
	FILE *in;
	char buff[512];
	if (!(in = popen("ps aux", "r"))) {
		return;

	}
	char user[8], stat[128], command[256];
	int pid, vsz, rss, tty, starth, startm, timem, times;
	float pcpu, pmem;
	ros_monitoring::Processes list;
	int i = 0;
	while (fgets(buff, sizeof(buff), in) != NULL) {
		sscanf(buff, "%s %d %f %f %d %d %s %s %d:%d %d:%d %s", &user, &pid,
				&pcpu, &pmem, &vsz, &rss, &tty, &stat, &starth, &startm, &timem,
				&times, &command);
		ROS_INFO("Cpu %f, %s", pcpu, command);
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

int main(int argc, char **argv) {

	ros::init(argc, argv, "cpu_monitor");
	ros::NodeHandle n("~");
	ros::Publisher avg_pub;
	ros::Publisher perc_pub;
	ros::Publisher proc_pub;

	float freq = 1;
	if (!n.getParam("frequency", freq))
	{
		ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
	}
	bool bAvarage = false;
	if (n.getParam("avarage", bAvarage)) {
		if (bAvarage) {
			avg_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/avg", 1);
		}
	}

	bool bPercent = false;
	if (n.getParam("percent", bPercent)) {
		if (bPercent) {
			perc_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/percent", 1);
		}
	}

	bool bProcesses = false;
	if (n.getParam("processes", bProcesses)) {
		if (bProcesses) {
			proc_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/allProcesses", 1);
		}
	}


	ros::Rate loop_rate(freq);
	CpuMonitor cpum;

	while (ros::ok()) {

		if (bAvarage) cpum.publishCpuUsage(perc_pub);
		if (bPercent) cpum.publishLoadAvg(avg_pub);
		if (bProcesses) cpum.publishProcessCpuUsage(proc_pub);
		loop_rate.sleep();
	}

}
