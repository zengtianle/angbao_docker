#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>//init_pos

#include <glog/logging.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
using namespace std;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg_){

 
    LOG(INFO) << "stamp:"<< msg_.header.stamp;
    LOG(INFO) << "frame_id:"<< msg_.header.frame_id;
    LOG(INFO) << "position:";
    LOG(INFO) << "       x: "<< msg_.pose.pose.position.x << "z: "<< msg_.pose.pose.position.y << "z: "<< msg_.pose.pose.position.z;
    LOG(INFO) << "orientation:";
    LOG(INFO) << "          x: "<< msg_.pose.pose.orientation.x << "z: "<< msg_.pose.pose.orientation.y << "z: "<< msg_.pose.pose.orientation.z << "w: "<< msg_.pose.pose.orientation.w;
    LOG(INFO) << "covariance:";
    LOG(INFO) << "          ["<< msg_.pose.covariance[0]<< ", "<< msg_.pose.covariance[1]<< ", "<< msg_.pose.covariance[2]<< ", "<< msg_.pose.covariance[3]<< ", "
                               << msg_.pose.covariance[4]<< ", "<< msg_.pose.covariance[5]<< ", "<< msg_.pose.covariance[6]<< ", "<< msg_.pose.covariance[7]<< ", " 
                               << msg_.pose.covariance[8]<< ", "<< msg_.pose.covariance[9]<< ", "<< msg_.pose.covariance[10]<< ", "<< msg_.pose.covariance[11]<< ", "
                               << msg_.pose.covariance[12]<< ", "<< msg_.pose.covariance[13]<< ", "<< msg_.pose.covariance[14]<< ", "<< msg_.pose.covariance[15]<< ", " 
                               << msg_.pose.covariance[16]<< ", "<< msg_.pose.covariance[17]<< ", "<< msg_.pose.covariance[18]<< ", "<< msg_.pose.covariance[19]<< ", "
                               << msg_.pose.covariance[20]<< ", "<< msg_.pose.covariance[21]<< ", "<< msg_.pose.covariance[22]<< ", "<< msg_.pose.covariance[23]<< ", " 
                               << msg_.pose.covariance[24]<< ", "<< msg_.pose.covariance[25]<< ", "<< msg_.pose.covariance[26]<< ", "<< msg_.pose.covariance[27]<< ", "
                               << msg_.pose.covariance[28]<< ", "<< msg_.pose.covariance[29]<< ", "<< msg_.pose.covariance[30]<< ", "<< msg_.pose.covariance[31]<< ", " 
                               << msg_.pose.covariance[32]<< ", "<< msg_.pose.covariance[33]<< ", "<< msg_.pose.covariance[34]<< ", "<< msg_.pose.covariance[35]<< "]" ;
    LOG(INFO);
    LOG(INFO);
    LOG(INFO);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "getInitialPose");
    ros::NodeHandle nh_, nh_private_("~");
    ROS_INFO("getInitialPose start!!!");

    ros::Subscriber sub_map_name = nh_.subscribe("/initialpose", 1,  poseCallback);
    string dir="/home/ob/.ob/log/LOGgetInitialPose";
	if (access(dir.c_str(), 0) == -1)
	{
		cout<<dir<<" is not existing"<<endl;
		cout<<"now make it"<<endl;
		int flag=mkdir(dir.c_str(),0777);
		if (flag == 0)
		{
			cout<<"make successfully"<<endl;
		} else {
			cout<<"make errorly"<<endl;
		}
    }


    google::InitGoogleLogging(argv[0]);
    
    FLAGS_log_dir = dir;
    FLAGS_logbufsecs = 0; 
    
    LOG(INFO) << "getInitialPose start!!!";

    ros::spin();
    google::ShutdownGoogleLogging();
    return 0;
}