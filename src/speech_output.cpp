#include "ros/ros.h"
#include "std_msgs/String.h"



void voiceCallback(std_msgs::String msg) {
  char command[200];
  ROS_INFO("%s", msg.data.c_str());
  sprintf(command, "espeak \"%s\"", msg.data.c_str());
  system(command);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speech_output");
    ros::NodeHandle n("~");

    ros::Subscriber voice_sub = n.subscribe("/voice_output", 5, voiceCallback);

    ros::spin();

}
