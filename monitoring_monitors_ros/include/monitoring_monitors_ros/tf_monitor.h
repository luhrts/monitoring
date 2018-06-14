#ifndef SRC_TFMONITOR_H_
#define SRC_TFSMONITOR_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "monitoring_core/monitor.h"

struct TransRequirement{
  std::vector<std::string> Frame_List;
  std::string Transform_Name;
  float errorlevel;
  float frequency;
  std::vector<double> TF_time_diff_max = std::vector<double>(1000);
};



struct TransInfo{
  std::vector<std::string> Frame_List;
  std::vector<std::string> Check_Result;
  float errorlevel=0;
  std::vector<ros::Duration> TF_time_diff;

};

/**
 * @brief The TF_Monitor class
 */
class TF_Monitor
{
public:
  TF_Monitor(ros::NodeHandle &n);
  ~TF_Monitor(){delete msg;}
  void GetTransform();
  tf::TransformListener tf_listener;
  void AddValueforMonitor();


private:
  /**
   * @brief loadConfig reads the parameter that are provided via the ros parameterserver and saves them to topicRequirements
   * @param n
   */
  void loadConfig(ros::NodeHandle &n);

  Monitor *msg;   ///< msg that saves information that will be published
  float freq;   ///< working frequency

  std::vector<TransRequirement> TransRequirements;
  std::vector<TransInfo> TransInfos;



   


};
#endif /* SRC_TF_Monitor_H_ */
