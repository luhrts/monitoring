/*
 * cpumonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#include "cpumonitor.h"



CpuMonitor::CpuMonitor() {
	// TODO Auto-generated constructor stub

}

CpuMonitor::~CpuMonitor() {
	// TODO Auto-generated destructor stub
}



void CpuMonitor::publishLoadAvg( ros::Publisher pub)
{
	double loadavg[3];
	std_msgs::Float32 avg;
	getloadavg(loadavg, 3); //works/updates around every 5 seconds
	//ROS_INFO(" %f, %f, %f", loadavg[0],loadavg[1],loadavg[2]);

	avg.data = loadavg[0];
	pub.publish(avg);
}


//-------------------------------------------------------




void CpuMonitor::init()
{
    FILE* file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser, &lastTotalUserLow,
        &lastTotalSys, &lastTotalIdle);
    fclose(file);
}

double CpuMonitor::getCurrentValue()
{
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

    if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow ||
        totalSys < lastTotalSys || totalIdle < lastTotalIdle){
        //Overflow detection. Just skip this value.
        percent = -1.0;
    }
    else{
        total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) +
            (totalSys - lastTotalSys);//if total is 0 a nan value is possible TODO!!!
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

void CpuMonitor::publishCpuUsage(ros::Publisher pub)
{
	if(lastTotalUser==0) {
		init();
	}
	double pCPU = getCurrentValue();
//	ROS_INFO("Load: %f ", pCPU);
	std_msgs::Float32 cpu_msg;
	cpu_msg.data = pCPU;
	pub.publish(cpu_msg);
}




int main( int argc, char **argv )
{

	ros::init(argc, argv, "cpu_monitor");
	ros::NodeHandle n;
	ros::Publisher avg_pub = n.advertise<std_msgs::Float32>("monitoring/cpu/avg", 1);
	ros::Publisher perc_pub = n.advertise<std_msgs::Float32>("monitoring/cpu/percentage", 1);
	ros::Rate loop_rate(1);
	CpuMonitor cpum;


	while(ros::ok()) {

		cpum.publishCpuUsage(perc_pub);
		cpum.publishLoadAvg(avg_pub);
		loop_rate.sleep();
	}

}
