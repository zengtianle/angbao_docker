/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Yongsheng_Zhang
 * @Date: 2021-06-28 11:06:06
 * @LastEditors: Yongsheng_Zhang
 * @LastEditTime: 2021-06-28 11:45:01
 */
#include <zys_localization/zys_localization.h>

ZysLocalization* Matching_;

// bool initialPoseCallback(zys_localization::localization_srv::Request &req,
//                         zys_localization::localization_srv::Response &res)
// {
//   // std::cout<<"x: "<<req.x <<"  y: "<<req.y<<std::endl;
//   ROS_INFO("get initial_pose x: %f,y: %f",req.x ,req.y);
//   bool result = Matching_->run(req.x, req.y);
//   res.result = result;

//   return result;
// }

bool initialPoseOneCallback(zys_localization::localization_srv::Request &req,
                        zys_localization::localization_srv::Response &res)
{
  // std::cout<<"x: "<<req.x <<"  y: "<<req.y<<std::endl;
  ROS_INFO("get initial_pose x: %f,y: %f, yaw: %f",req.x ,req.y,req.yaw);
  timeval tStart,cTime;
  gettimeofday(&tStart, 0);

  bool result;
  Matching_->runone(req.x, req.y, req.yaw, result, res.score);
  res.result = result;

  gettimeofday(&cTime, 0);
  cTime.tv_sec -= tStart.tv_sec;
  cTime.tv_usec -= tStart.tv_usec;
  cout<<"time = "<<((1000000LL * cTime.tv_sec + cTime.tv_usec)/1000)<<"ms"<<endl;
  return result;
}

bool initialPoseScoreCallback(zys_localization::localization_srv::Request &req,
                        zys_localization::localization_srv::Response &res)
{
  // std::cout<<"x: "<<req.x <<"  y: "<<req.y<<std::endl;
  ROS_INFO("score initial_pose x: %f,y: %f, yaw:%f",req.x ,req.y ,req.yaw);
  timeval tStart,cTime;
  gettimeofday(&tStart, 0);
  bool result;
  Matching_->score(req.x, req.y, req.yaw, result, res.score);
  res.result = result;
  gettimeofday(&cTime, 0);
  cTime.tv_sec -= tStart.tv_sec;
  cTime.tv_usec -= tStart.tv_usec;
  cout<<"time = "<<((1000000LL * cTime.tv_sec + cTime.tv_usec)/1000)<<"ms"<<endl;
  return result;
}
void initialPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  ROS_INFO("get initial_pose x: %f,y: %f, yaw:%f",msg.pose.pose.position.x ,msg.pose.pose.position.y,tf::getYaw(msg.pose.pose.orientation) / M_PI *180);
  timeval tStart,cTime;
  gettimeofday(&tStart, 0);
  bool result;
  float score;
  Matching_->runone(msg.pose.pose.position.x, msg.pose.pose.position.y, tf::getYaw(msg.pose.pose.orientation) / M_PI *180, result, score);
  gettimeofday(&cTime, 0);
  cTime.tv_sec -= tStart.tv_sec;
  cTime.tv_usec -= tStart.tv_usec;
  cout<<"time = "<<((1000000LL * cTime.tv_sec + cTime.tv_usec)/1000)<<"ms"<<endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "zys_localization");
  ros::NodeHandle nh_;
  ROS_INFO("zys_localization strat!!!");
  ZysLocalization Matching;
  Matching_ = &Matching;
  ros::Subscriber sub_initialPose_ = nh_.subscribe("initial_pose", 1, initialPoseCallback_);
  // ros::ServiceServer service = nh_.advertiseService("initial_pose", initialPoseCallback);
  ros::ServiceServer service_one = nh_.advertiseService("initial_pose_one", initialPoseOneCallback);
  ros::ServiceServer service_score = nh_.advertiseService("initial_pose_score", initialPoseScoreCallback);
  ros::spin();

  return 0;
}
