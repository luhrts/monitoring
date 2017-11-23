/*
 * nodemonitor.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: matthias
 */

#include "nodemonitor.h"

NodeMonitor::NodeMonitor() {

}
NodeMonitor::NodeMonitor(std::string filename) {
	nodesList = readFile(filename);

}

NodeMonitor::~NodeMonitor() {
	// TODO Auto-generated destructor stub
}

std::vector<std::string> NodeMonitor::getNodeList() {
	ros::V_string nodes;
	ros::master::getNodes(nodes);
	std::vector<std::string> ret;
	for(int i=0; i<nodes.size();i++) {
//		ROS_INFO("%s", nodes[i].c_str());
		ret.push_back(nodes[i]);
	}
	return ret;
}

void generateFile(char* filename) {
	FILE* file;
	file = fopen(filename, "w");
	NodeMonitor nodeMon;
	std::vector<std::string> nodes = nodeMon.getNodeList();
	for(int i=0; i<nodes.size();i++) {
		fprintf(file, "%s\n", nodes[i].c_str());
	}
	fclose(file);
	ROS_INFO("Found %d Nodes", nodes.size());
}

std::vector<std::string> NodeMonitor::readFile(std::string filename) {
	FILE* file;
	file = fopen(filename.c_str(), "r");
	char line[256];
	std::vector<std::string> ret;
	while (fgets(line, sizeof(line), file) != NULL) {
		ret.push_back(line);
	}
	return ret;
}



int main(int argc, char **argv) {

	ros::init(argc, argv, "node_monitor");
	ros::NodeHandle n("~");
	ros::Publisher monitor_pub = n.advertise<ros_monitoring::MonitoringInfo>(
			"/monitoring/all", 1);


	if( argc > 1){
		if( !strcmp(argv[1],"-generate")){
			ROS_INFO("Fetching Nodes and saving to %s", argv[2]);
			generateFile(argv[2]);
		} else {
			ROS_ERROR("%s is not a valid option, try -generate $file", argv[1]);
		}
		return 0;
	}

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}

	std::string filename = "nodes.txt";

	if (!n.getParam("nodefile", filename)) {
		ROS_WARN("No Network Interface configured. Using %s",
				filename.c_str());
	}



	NodeMonitor nodeMon(filename);
	ros::Rate loop_rate(freq);

	while(ros::ok()) {
		nodeMon.getNodeList();


		loop_rate.sleep();
	}
}
