#include <ros/ros.h>
#include "zys_localization/localization_srv.h"






int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<zys_localization::localization_srv>("initial_pose");
 
	zys_localization::localization_srv srv;
	srv.request.x = 0.5;
	srv.request.y = 0.5;
	srv.request.yaw = 0;


	if(client.call(srv))
	{
		ROS_INFO("service success. result : %d", srv.response.result);
	}
	else
	{
		ROS_INFO("service failed!");
		return 1;
	}
 
	return 0;
}
