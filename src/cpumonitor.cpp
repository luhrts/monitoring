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
/**
 * getting all values of the cpu at first because they are incremel. They are
 * catched for every core aswell and saved to a vector.
 */
void CpuMonitor::init() {

	int numCPU = sysconf(_SC_NPROCESSORS_ONLN);

	ROS_ERROR("number: %d", numCPU);
	unsigned long long lastTotalUser_, lastTotalUserLow_, lastTotalSys_,
			lastTotalIdle_;

	FILE* file = fopen("/proc/stat", "r");
	fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser_, &lastTotalUserLow_,
			&lastTotalSys_, &lastTotalIdle_);

	lastTotalUser.push_back(lastTotalUser_);
	lastTotalUserLow.push_back(lastTotalUserLow_);
	lastTotalSys.push_back(lastTotalSys_);
	lastTotalIdle.push_back(lastTotalIdle_);
	char buff[256];
	int n;

	fgets(buff, sizeof(buff), file); //to jump over the first line (already done thatone)
	while (fgets(buff, sizeof(buff), file) != NULL) {

		sscanf(buff, "cpu%d %llu %llu %llu %llu", &n, &lastTotalUser_,
				&lastTotalUserLow_, &lastTotalSys_, &lastTotalIdle_);

		lastTotalUser.push_back(lastTotalUser_);
		lastTotalUserLow.push_back(lastTotalUserLow_);
		lastTotalSys.push_back(lastTotalSys_);
		lastTotalIdle.push_back(lastTotalIdle_);

		if (n == numCPU - 1) { //read last cpu, stop now
			break;
		}
	}
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

	if (totalUser < lastTotalUser[0] || totalUserLow < lastTotalUserLow[0]
			|| totalSys < lastTotalSys[0] || totalIdle < lastTotalIdle[0]) {
		//Overflow detection. Just skip this value.
		percent = -1.0;
	} else {
		//because the value is incrementel
		total = (totalUser - lastTotalUser[0])
				+ (totalUserLow - lastTotalUserLow[0])
				+ (totalSys - lastTotalSys[0]);	//if total is 0 a nan value is possible TODO!!!
//        ROS_INFO("total: %llu", total);
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

double CpuMonitor::getCPUCoreLoad(int n) { //TODO need to test if this is the right cpu core load
	double percent;
	FILE* file;
	unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

	file = fopen("/proc/stat", "r");

	char buffer[256];
	for (int i = 0; i <= n; i++) {
		fgets(buffer, 256, file);
	}
	fscanf(file, "cpu%d %llu %llu %llu %llu", &n, &totalUser, &totalUserLow,
			&totalSys, &totalIdle);
	fclose(file);

	if (totalUser < lastTotalUser[n] || totalUserLow < lastTotalUserLow[n]
			|| totalSys < lastTotalSys[n] || totalIdle < lastTotalIdle[n]) {
		//Overflow detection. Just skip this value.
		percent = -1.0;
	} else {
		//because the value is incrementel
		total = (totalUser - lastTotalUser[n])
				+ (totalUserLow - lastTotalUserLow[n])
				+ (totalSys - lastTotalSys[n]);	//if total is 0 a nan value is possible TODO!!!
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
	ROS_ERROR("CPU %d: %f", n, percent);
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

void CpuMonitor::publishCPUTemp(ros::Publisher pub) {
	FILE* file;
	file = fopen("/sys/class/hwmon/hwmon1/temp1_input", "r");
	int input = 0;
	fscanf(file, "%d", &input);
	fclose(file);
	float temp = input / 1000; //in °C
	std_msgs::Float32 temp_msg;
	temp_msg.data = temp;
	pub.publish(temp_msg);

}

//-------------------------------------------------------------

int main(int argc, char **argv) {

	ros::init(argc, argv, "cpu_monitor");
	ros::NodeHandle n("~");
	ros::Publisher avg_pub, temp_pub, perc_pub, proc_pub;

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
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
			perc_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/percent",
					1);
		}
	}

	bool bProcesses = false;
	if (n.getParam("processes", bProcesses)) {
		if (bProcesses) {
			proc_pub = n.advertise<std_msgs::Float32>(
					"/monitoring/cpu/allProcesses", 1);
		}
	}
	bool bTemp = false;
	if (n.getParam("temperature", bTemp)) {
		if (bTemp) {
			temp_pub = n.advertise<std_msgs::Float32>("/monitoring/cpu/temp",
					1);
		}
	}

	ros::Rate loop_rate(freq);
	CpuMonitor cpum;

	while (ros::ok()) {

		if (bPercent)
			cpum.publishCpuUsage(perc_pub);
		if (bAvarage)
			cpum.publishLoadAvg(avg_pub);
		if (bProcesses)
			cpum.publishProcessCpuUsage(proc_pub);
		if (bTemp)
			cpum.publishCPUTemp(temp_pub);


		cpum.getCPUCoreLoad(1);
		cpum.getCPUCoreLoad(2);


		loop_rate.sleep();
	}

}
