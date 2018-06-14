#include"monitoring_monitors_ros/tf_monitor.h"
void TF_Monitor::loadConfig(ros::NodeHandle &n) {
  freq = 10;
  if (!n.getParam("monitoring/frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }
  std::vector<std::string> Transform_Name;
  std::vector<std::string> Frame_List;
  if(n.getParam("Transform_check", Transform_Name)) {
    for(std::string name: Transform_Name) {
      TransRequirement tr;
      tr.Transform_Name=name;
      if(!n.getParam(name + "/Frame_List", tr.Frame_List)) {
        ROS_ERROR("%s tf: No Frame_List supplied.", name.c_str());
      }
      if(!n.getParam(name + "/errorlevel", tr.errorlevel)) {
        ROS_ERROR("%s tf: No errorlevel supplied.", name.c_str());
      }
      TransRequirements.push_back(tr);
    }
  }
}
void TF_Monitor::AddValueforMonitor(){
    for(int i=0;i<TransInfos.size();i++){
       TransInfo TI;
       TI=TransInfos[i];
//addValue for ros monitore
       std::string Frame_List="";
       for(int j=0;j<TI.Frame_List.size();j++)
       {
           Frame_List=Frame_List+ "," + TI.Frame_List[j];
       }

       msg->addValue("Frame_List: ", Frame_List, "", 0.0);
       if(!TI.errorlevel == 0){
        msg->addValue("Transform Error in Frame_List",0.0,"",TI.errorlevel);
       }

       for(int k=0;k<TI.Check_Result.size();k++){
           msg->addValue("Check_Result: ",TI.Check_Result[k], "", 0.0);
           msg->addValue("Time Delay: ",TransInfos[i].TF_time_diff[k].toSec(), "Sec", 0.0);
           if(TransInfos[i].TF_time_diff[k].toSec()>TransRequirements[i].TF_time_diff_max[k]){
                TransRequirements[i].TF_time_diff_max[k]=TransInfos[i].TF_time_diff[k].toSec();
           }
           msg->addValue("Max Time Delay: ",TransRequirements[i].TF_time_diff_max[k], "Sec", 0.0);

       }


    }




}

 void TF_Monitor::GetTransform(){
       for(int i=0;i<TransRequirements.size();i++){
       TransInfo TI;
       TI.Frame_List = TransRequirements[i].Frame_List;
       for(int j=0;j<TransRequirements[i].Frame_List.size();j++){
       for(int k=j+1;k<TransRequirements[i].Frame_List.size();k++){
       tf::StampedTransform  st;
       std::string Check_Result;
       ros::Duration TF_time_diff;
       bool Error=false;
       //Check Transform and get Delay
       try{
          tf_listener.lookupTransform(TransRequirements[i].Frame_List[j], TransRequirements[i].Frame_List[k],ros::Time(0), st);
       }
       catch (tf::TransformException ex){
           Error=true;
           TI.errorlevel=TransRequirements[i].errorlevel;
          }

    if(!Error){
          Check_Result="There is a transform between "+TransRequirements[i].Frame_List[k]+" and "+TransRequirements[i].Frame_List[j];
          TI.Check_Result.push_back(Check_Result);
          TI.TF_time_diff.push_back(ros::Time::now()- st.stamp_);

       }
       else{

           Check_Result = " No transform between "+TransRequirements[i].Frame_List[k]+" and "+TransRequirements[i].Frame_List[j];
           TI.TF_time_diff.push_back(ros::Duration(0));
           TI.Check_Result.push_back(Check_Result);
       }
       }
       }
       TransInfos.push_back(TI);
       }

 }


  TF_Monitor::TF_Monitor(ros::NodeHandle &n) {
  n.setParam("/enable_tf", true);
  loadConfig(n);
  msg = new Monitor (n, "tf for Ros" );
  ros::Rate loop_rate(freq);

  while(n.ok()) {
    TF_Monitor::GetTransform();
    AddValueforMonitor();
    //clean old TransInfo
    std::vector<TransInfo> NewTransInfos;
    TransInfos=NewTransInfos;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tf_monitor");
  ros::NodeHandle n("~");

  TF_Monitor tf_moni(n);
  return 0;
}
