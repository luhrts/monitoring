#include "monitoring_monitors_ros/node_ressource_monitor_node.h"


std::string getLocalIPs(){
    std::string ips = "";
    struct ifaddrs *ifaddr, *ifa;
    int family, s, n;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        freeifaddrs(ifaddr);
        return ips;
    }

    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        if (family == AF_INET) {
            s = getnameinfo(ifa->ifa_addr,
                (family == AF_INET) ? sizeof(struct sockaddr_in) :
                sizeof(struct sockaddr_in6),
                host, NI_MAXHOST,
                NULL, 0, NI_NUMERICHOST);
            if (s != 0) {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                continue;
            }
            ips = ips + ":" +host;
            printf("\t\taddress: <%s>\n", host);
        }else{
            continue;
        }
    }
    freeifaddrs(ifaddr);
    if(ips.size() > 0)
        ips.erase(0, 1);
    return ips;
}

std::string getLocalInterfaces(){
    std::string interfaces;
    struct ifaddrs *ifaddr, *ifa;
    int family, s, n;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        freeifaddrs(ifaddr);
        return interfaces;
    }

    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
        if (ifa->ifa_addr == NULL)
            continue;
        family = ifa->ifa_addr->sa_family;
        if (family == AF_INET) {
            interfaces = interfaces + ":" +ifa->ifa_name;
            printf("\t interface: <%-8s>\n", ifa->ifa_name);
        }else{
            continue;
        }
    }
    freeifaddrs(ifaddr);
    if(interfaces.size() > 0)
        interfaces.erase(0, 1);
    return interfaces;
}

NodeResMon::NodeResMon(ros::NodeHandle &n) : loop_rate(1) {
    statFormatStr = "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %ld %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %d %d %u %u %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %d";
    ips = getLocalIPs();
    interfaces = getLocalInterfaces();
    if (ips.size() == 0 && interfaces.size() == 0)
        ROS_ERROR("Couldn't get info of local machine. The Node_ressource_monitor wont work");

    aggregation = AggregationStrategies::LAST;
    loadConfig(n);

    msg = new Monitor (n, "node_ressource_monitoring");
    loop_rate = ros::Rate(freq);


    checkNewNodes();
    nodeUpdateTime = ros::Time::now();
    page_size_kb = sysconf(_SC_PAGE_SIZE);
    clock_ticks_per_sec = sysconf(_SC_CLK_TCK);
    init();
}

NodeResMon::~NodeResMon()
{
    delete msg;
}

void NodeResMon::init()
{
    updateNodes(true);
}

void NodeResMon::updateNodeValues(){
    getNodeInfos();
    publishNodeInfos();
}

void NodeResMon::publishNodeInfos()
{
    //ROS_INFO("Publishing");
    std::string key;
    struct PidStat v;
    for(auto const& x : node_map)
    {
        key = x.first; //pid
        v = x.second;
        char buff[256];
        std::string name(v.comm);
        //ROS_INFO("Node Name: %s", name.c_str());
        sprintf(buff, "%s/%s", v.comm, "name");
        msg->addValue(std::string(buff), name, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "pid");
        msg->addValue(std::string(buff), v.pid, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "user_time");
        msg->addValue(std::string(buff), v.user_t, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "system_time");
        msg->addValue(std::string(buff), v.sys_t, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "cpu_percent");
        msg->addValue(std::string(buff), v.cpu_perc, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "state");
        std::string state;
        switch (v.state)
        {
            case 'R':
                state = "Running";
                break;
            case 'S':
                state = "Sleeping";
                break;
            case 'D':
                state = "Waiting";
                break;
            case 'Z':
                state = "Zombie";
                break;
            case 'T':
                state = "Stopped";
                break;
            case 't':
                state = "Tracing_stop";
                break;
            case 'X':
                state = "Dead";
                break;
            case 'x':
                state = "Dead";
                break;
            case 'K':
                state = "Wakekill";
                break;
            case 'W':
                state = "WakingORPaging";
                break;
            case 'P':
                state = "Parked";
                break;
            default:
                state = "unknown_state" + v.state;
                break;
        }
        msg->addValue(std::string(buff), state, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "minor_faults");
        msg->addValue(std::string(buff), v.minflt, "", 0, aggregation);//minflt
        sprintf(buff, "%s/%s", v.comm, "major_faults");
        msg->addValue(std::string(buff), v.majflt, "", 0, aggregation);//majflt
        sprintf(buff, "%s/%s", v.comm, "priority");
        msg->addValue(std::string(buff), v.priority, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "num_threads");
        msg->addValue(std::string(buff), v.num_threads, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "vmem_size"); // in bytes
        msg->addValue(std::string(buff), v.vsize, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "rss_mem_size"); // in bytes
        msg->addValue(std::string(buff), v.rss, "", 0, aggregation);
        // TODO: instruction- and stackpointer ?? kstkeip kstkesp
        // TODO: infos about signals ??
        
        sprintf(buff, "%s/%s", v.comm, "cpu_num");
        msg->addValue(std::string(buff), v.processor, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "scheduling_policy");
        msg->addValue(std::string(buff), v.policy, "", 0, aggregation);
        //TODO: Aggregated io delays ?? delayacct_blkio_ticks
        
        sprintf(buff, "%s/%s", v.comm, "voluntary_ctxt_switches");
        msg->addValue(std::string(buff), v.voluntary_ctxt_switches, "", 0, aggregation);
        sprintf(buff, "%s/%s", v.comm, "nonvoluntary_ctxt_switches");
        msg->addValue(std::string(buff), v.nonvoluntary_ctxt_switches, "", 0, aggregation);
    }
}


int NodeResMon::cpuCount()
{
    std::string line;
    std::ifstream file("/proc/cpuinfo");
    int num = 0;
    std::size_t found;
    while(getline(file, line))
    {
        found = line.find("processor", 0, 10); // only search in the beginning of a line
        if(found != std::string::npos)
            num++;
    }
    file.close();
    return num;
}

void NodeResMon::checkNewNodes(){
    // TODO
    /*ros::V_string nodeNames;
    ros::master::getNodes(&nodeNames);*/

    // implement a service in python
    // srv gets list of interfaces and/or ip's
    // responds with string of nodes / string of PID's
    // e.g "node_1 node_2 ..." and "pid_1 pid_2 ..." !the strings are space separated
    // parse the strings and safe as class-var
    std::string test = "12041 11938";
    std::stringstream ss(test);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    std::copy(vstrings.begin(), vstrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    pid_s = vstrings;
    //node_s = res.nodes;
}

void NodeResMon::getNodeInfos(){
    ros::Time now = ros::Time::now();
    if ((now - nodeUpdateTime).toSec() > checkNewNodesUpdateIntervall){
        checkNewNodes();
        nodeUpdateTime = now;
    }

    updateNodes(false);
}

void NodeResMon::updateNodes(bool init){
    // this Function reads from /proc/<pid>/stat to get infos of a process
    struct dirent *entry = NULL;
    DIR *dp = NULL;

    dp = opendir("/proc");
    if (dp != NULL)
    {
        while ((entry = readdir(dp)))
        { // iterate through proc
            //printf("open Dir %s\n", entry->d_name);
            std::regex re("^[0-9]+$");
            if(std::regex_search(entry->d_name, re)) // check if d_name is a pid
            {
                bool found = false;
                for(auto const& pid: pid_s)
                {
                    if(strcmp(pid.c_str(),entry->d_name) == 0)
                    {
                        found = true;
                        break;
                    }
                }

                if(found)
                {

                    struct PidStat ps;
                    // the pid is a local rosnode
                    // read the values from dir /proc/<pid>/stat
                    std::string line;
                    std::ostringstream filename;
                    filename << "/proc/"<<entry->d_name<<"/stat";
                    //sprintf(line,"/proc/%s/stat",entry->d_name)
                    std::ifstream file(filename.str());
                    //printf("%s Is a rosnode \n", filename.str().c_str());
                    if(file.is_open())
                    {
                        // stat-files only have one line
                        if(getline(file, line))
                        {
                            //printf("parsing line content \t");
                            //printf("%s\n", line.c_str());
                            readargs(line.c_str(), statFormatStr.c_str(), &ps.pid, &ps.comm, &ps.state, &ps.ppid,
                                &ps.pgrp, &ps.session, &ps.tty_nr, &ps.tpgid, &ps.flags, &ps.minflt,
                                &ps.cminflt, &ps.majflt, &ps.cmajflt, &ps.utime, &ps.stime,
                                &ps.cutime, &ps.cstime, &ps.priority, &ps.nice, &ps.num_threads,
                                &ps.itrealvalue, &ps.starttime, &ps.vsize, &ps.rss, &ps.rsslim,
                                &ps.startcode, &ps.encode, &ps.startstack, &ps.kstkesp, &ps.kstkeip,
                                &ps.signal, &ps.blocked, &ps.sigignore, &ps.sigcatch, &ps.wchan,
                                &ps.nswap, &ps.cnswap, &ps.exit_sig, &ps.processor, &ps.rt_priority,
                                &ps.policy, &ps.delayacct_blkio_ticks, &ps.guest_time, &ps.cguest_time,
                                &ps.start_data, &ps.end_data, &ps.start_brk, &ps.arg_start, &ps.arg_end,
                                &ps.env_start, &ps.env_end, &ps.exit_code);

                            /*sscanf(line.c_str(), statFormatStr.c_str(), ps.pid, ps.comm, ps.state, ps.ppid,
                                ps.pgrp, ps.session, ps.tty_nr, ps.tpgid, ps.flags, ps.minflt,
                                ps.cminflt, ps.majflt, ps.cmajflt, ps.utime, ps.stime,
                                ps.cutime, ps.cstime, ps.priority, ps.nice, ps.num_threads,
                                ps.itrealvalue, ps.starttime, ps.vsize, ps.rss, ps.rsslim,
                                ps.startcode, ps.encode, ps.startstack, ps.kstkesp, ps.kstkeip,
                                ps.signal, ps.blocked, ps.sigignore, ps.sigcatch, ps.wchan,
                                ps.nswap, ps.cnswap, ps.exit_sig, ps.processor, ps.rt_priority,
                                ps.policy, ps.delayacct_blkio_ticks, ps.guest_time, ps.cguest_time,
                                ps.start_data, ps.end_data, ps.start_brk, ps.arg_start, ps.arg_end,
                                ps.env_start, ps.env_end, ps.exit_code); // times are measured in clock_ticks */
                            ps.user_t = ps.utime / clock_ticks_per_sec;
                            ps.sys_t  = ps.stime / clock_ticks_per_sec;
                            //printf("Done parsing arguments\n");
                            // see http://man7.org/linux/man-pages/man5/proc.5.html for infos

                            if(!init)
                            {
                                //calculate cpu_percent

                                double delta_t = (ros::Time::now() - node_map[entry->d_name].last_u_time).toSec();
                                double delta_p =  (ps.user_t - node_map[entry->d_name].user_t)
                                                + (ps.sys_t - node_map[entry->d_name].sys_t);
                                float cpu_percent;
                                try
                                {
                                    cpu_percent = (delta_p / delta_t) * 100; // this is avg cpu usage of the process spread over all cpu cores
                                } catch(...)
                                {
                                    //Division by zero
                                    cpu_percent = 0.0;
                                }
                                ps.cpu_perc = cpu_percent * ps.processor;
                            }
                        }
                        file.close();
                        //printf("stat file closed");
                    }
                    std::ostringstream status_file;
                    status_file << "/proc/" << entry->d_name << "/status";
                    std::regex vol_re("voluntary_ctxt_switches:\t(\d+)");
                    std::regex nonvol_re("nonvoluntary_ctxt_switches:\t(\d+)");
                    std::ifstream file_s(status_file.str());
                    //printf("Opening status-file %s", status_file.str().c_str());
                    if(file_s.is_open())
                    {
                        while(getline(file_s, line))
                        {
                            if(std::regex_search(line, vol_re))
                            {
                                std::string temp = line.substr(line.find(":") + 1);
                                temp.erase(std::remove_if(temp.begin(), temp.end(), [](unsigned char x){return std::isspace(x);}), temp.end());
                                ps.voluntary_ctxt_switches = std::strtoul(temp.c_str(), NULL, 10); // convert to unsigned long interger
                            }
                            if(std::regex_search(line, nonvol_re))
                            {
                                std::string temp = line.substr(line.find(":") + 1);
                                temp.erase(std::remove_if(temp.begin(), temp.end(), [](unsigned char x){return std::isspace(x);}), temp.end());
                                ps.nonvoluntary_ctxt_switches = std::strtoul(temp.c_str(), NULL, 10); // convert to unsigned long interger
                            }
                        }
                        file_s.close();
                    }
                    ps.last_u_time = ros::Time::now();
                    node_map[entry->d_name] = ps;
                } // the dir name was a pid of a rosnode
            }
            //printf ("%s\n", entry->d_name);
        }
    }
    closedir(dp);
}

void NodeResMon::readargs(const char *buff, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int count = vsscanf(buff, fmt, ap);
    //printf("%d args", count);
    va_end(ap);
}

void NodeResMon::loadConfig(ros::NodeHandle &n) {
  freq = 50;
  if (!n.getParam("monitoring/frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
  }

  if(n.getParam("monitor_mode", monitor_mode)) {

    switch (monitor_mode){
    case 1 :
      aggregation = AggregationStrategies::LAST;
      break;
    case 2 :
      aggregation = AggregationStrategies::FIRST;
      break;
    case 3 :
      aggregation = AggregationStrategies::MIN;
      break;
    case 4 :
      aggregation = AggregationStrategies::MAX;
      break;
    case 5 :
      aggregation = AggregationStrategies::AVG;
      break;

    }
  }

  checkNewNodesUpdateIntervall = 3.0;
  if (!n.getParam("newNodesUpdateIntervall", checkNewNodesUpdateIntervall))
  {
    ROS_WARN("No update time to check for new nodes. Working with %f sec.", checkNewNodesUpdateIntervall);
  }


/*  std::vector<std::string> topics;
  if(n.getParam("topics", topics)) {
    for(std::string name: topics) {
      TopicRequirement tr;

      if(!n.getParam(name + "/topic", tr.topic)) {
        ROS_ERROR("%s Statistics: No topic supplied.", name.c_str());
      } else {
        if(tr.topic.front() != '/') {
          tr.topic.insert(0, "/");
        }
      }

      if(!n.getParam(name + "/source", tr.source)) {
        ROS_ERROR("%s Statistics: No source supplied.", name.c_str());
      }else {
        if(tr.source.front() != '/') {
          tr.source.insert(0, "/");
        }
      }
      if(!n.getParam(name + "/destination", tr.destination)) {
        ROS_ERROR("%s Statistics: No destination supplied.", name.c_str());
      }else {
        if(tr.destination.front() != '/') {
          tr.destination.insert(0, "/");
        }
      }
      if(!n.getParam(name + "/frequency", tr.frequency)) {
        ROS_ERROR("%s Statistics: No frequency supplied.", name.c_str());
      }
      if(!n.getParam(name + "/size", tr.size)) {
        ROS_ERROR("%s Statistics: No size supplied.", name.c_str());
      }
      if(!n.getParam(name + "/dFrequency", tr.dFrequency)) {
        ROS_ERROR("%s Statistics: No dFrequency supplied.", name.c_str());
      }
      if(!n.getParam(name + "/dSize", tr.dSize)) {
        ROS_ERROR("%s Statistics: No dSize supplied.", name.c_str());
      }
      if(!n.getParam(name + "/type", tr.type)) {
        ROS_ERROR("%s Statistics: No type supplied.", name.c_str());
      }
      if(!n.getParam(name + "/errorlevel", tr.errorlevel)) {
        ROS_ERROR("%s Statistics: No errorlevel supplied.", name.c_str());
      }
      topicRequirements.push_back(tr);
    }
  }*/
}


void NodeResMon::run(){
    ros::spinOnce();
    loop_rate.sleep();
    updateNodeValues();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "node_ressource_monitor_node");
  ros::NodeHandle n("~");

  NodeResMon sm(n);
  while(ros::ok())
  {
    sm.run();
  }
  return 0;
}
