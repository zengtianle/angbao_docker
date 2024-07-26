#!/usr/bin/env python
#encoding=utf-8

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid 

from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8,Int16, Int32, UInt16, Float32, String, Bool,Byte
import tf
import numpy
import math
from obrobot_navigation.msg import PoseDefine,PoseOperation
from obrobot_navigation.srv import PoseManage


from map_server.srv import reset_map

from std_srvs.srv import Empty
from zys_localization.srv import localization_srv

class Relocation():
    def __init__(self):
        
        rospy.loginfo("......ob_relocation.py is running.....")
        rospy.loginfo("......start_relocation.....")  

        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = "/map"
        self.initial_pose.pose.pose.position.x = 0
        self.initial_pose.pose.pose.position.y = 0
        self.initial_pose.pose.pose.orientation.z = 0
        self.initial_pose.pose.pose.orientation.w = 0
        self.initial_pose.pose.covariance[0] = 0.1 #0.5
        self.initial_pose.pose.covariance[7] = 0.1 #0.5
        self.initial_pose.pose.covariance[35] = 0.0685389194520094 #15deg


        self.ob_error_status = 0
        self.relocation_finish_flag = False
        self.in_recharing = 0
        rospy.Subscriber('recharge_status', Int16, self.recharge_status_cb)
        self.recharge_status = 0
        self.ob_current_floor_pub = rospy.Publisher("ob_current_floor", Int16, queue_size = 5, latch=True)
        rospy.Subscriber('ob_error_status_driver', Int16, self.ob_error_status_driver_cb)       
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 5)
        self.ob_relocation_finish_pub = rospy.Publisher("ob_relocation_finish", Bool, queue_size = 5, latch=True)
        #发送机器人定位状态话题：定位成功为true,定位失败为false
        self.ob_location_status_pub = rospy.Publisher("ob_location_status", Bool, queue_size = 5)
        self.ob_exit_recharge_pub = rospy.Publisher("ob_exit_recharge", Byte, queue_size = 5)
        self.ob_web_tool_runing_pub = rospy.Publisher("ob_web_tool_runing", Bool, queue_size = 5, latch=True)
        self.ob_changing_current_map_pub = rospy.Publisher('ob_changing_current_map', Bool, queue_size = 5) # 切换地图中
        self.ob_map_goal_id_pub = rospy.Publisher("ob_map_goal_id", Int32, queue_size = 5)
 
        rospy.Subscriber("recharge_handle", Int16, self.recharge_handle_cb)

        rospy.logdebug("......start_relocation.....")
        rospy.sleep(3)
        poseoperation=PoseOperation()
        poseoperation.opt   = "select"
        poseoperation.id    = 100000
        # rospy.loginfo("......waiting posemanage_server.....")
        rospy.wait_for_service('posemanage_server')
        # rospy.loginfo("......posemanage_server start.....")

        try:
            posemanage = rospy.ServiceProxy('posemanage_server', PoseManage)
            resp = posemanage(poseoperation)
            if "SUCCESS" in resp.status:
                self.initial_pose.pose.pose.position.x = resp.posedefine.pose.position.x
                self.initial_pose.pose.pose.position.y = resp.posedefine.pose.position.y
                self.initial_pose.pose.pose.orientation.x = resp.posedefine.pose.orientation.x
                self.initial_pose.pose.pose.orientation.y = resp.posedefine.pose.orientation.y
                self.initial_pose.pose.pose.orientation.z = resp.posedefine.pose.orientation.z
                self.initial_pose.pose.pose.orientation.w = resp.posedefine.pose.orientation.w
                print(resp.status)
            else:
                print("cannot get the initial pose")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        #切换到1楼地图
        self.ob_changing_current_map_pub.publish(True)
        self.exp_floor = 1
        self.reset_map = rospy.ServiceProxy('reset_map', reset_map)
        # rospy.loginfo("ob_relocation.py wait for reset_map")
        self.reset_map.wait_for_service()
        # rospy.loginfo('found "reset_map" service')
        floor_test_src = "/home/ob/.ob/map/OB_floor_" + str(self.exp_floor) + ".yaml"  #目标楼层
        tempresult = self.reset_map(floor_test_src, "map")
        floor_test_src = "/home/ob/.ob/map/OB_floor_location_" + str(self.exp_floor) + ".yaml"  #目标楼层
        tempresult = self.reset_map(floor_test_src, "map_amcl")
        if tempresult.result is True:
            self.currentfloor = int(self.exp_floor)
            self.ob_current_floor_pub.publish(self.currentfloor)
            rospy.loginfo("self.currentfloor:"+str(self.currentfloor))
      
        #等待地图切换完成后才能重定位     
        rospy.sleep(3)

        # 将当前位置设置为开机位置（也即是充电桩位置），进行地图重定位

        temp_initial_pose = PoseWithCovarianceStamped()
        temp_initial_pose.header.frame_id = "/map"
        temp_initial_pose.pose.covariance[0] = 0.1 #0.5
        temp_initial_pose.pose.covariance[7] = 0.1 #0.5
        temp_initial_pose.pose.covariance[35] = 0.0685389194520094 #15deg
        temp_initial_pose.pose.pose = resp.posedefine.pose
        self.ob_changing_current_map_pub.publish(True)
        rospy.sleep(1)
        self.initialpose_pub.publish(temp_initial_pose)
        # rospy.loginfo("recharge_addback: publish topic /initialpose")
        # rospy.loginfo("recharge: relocation finished")
        rospy.sleep(1)
        self.ob_changing_current_map_pub.publish(False)


        # #启动一次YS重定位操作
        # #先求出方位角
        # _rot = [self.initial_pose.pose.pose.orientation.x, self.initial_pose.pose.pose.orientation.y, self.initial_pose.pose.pose.orientation.z, self.initial_pose.pose.pose.orientation.w]
        # self.yaw_cur = tf.transformations.euler_from_quaternion(_rot)[2]
        # self.yaw_cur = self.yaw_cur/math.pi*180 #弧度转角度

        # self.ob_localization = rospy.ServiceProxy('initial_pose_one', localization_srv)
        # rospy.loginfo("relocation wait for ob_localization")
        # self.ob_localization.wait_for_service()
        # rospy.loginfo('found "ob_localization" service')
        # self.ob_changing_current_map_pub.publish(True)
        # # rospy.sleep(0.3)
        # rospy.loginfo('start localization!!')
        # tempresult = self.ob_localization(self.initial_pose.pose.pose.position.x,self.initial_pose.pose.pose.position.y,self.yaw_cur)
        # if tempresult.result is True:
        #     rospy.loginfo("ob_localization finish,score："+ str(tempresult.score))
        # self.ob_changing_current_map_pub.publish(False)

        rospy.loginfo("power on relocation: relocation finished")
        
        rospy.sleep(3)
        #机器人一定要放置在充电桩上开机才能定位成功，判断条件就是是否正在充电
        if self.recharge_status >=3:
            self.relocation_finish_flag = True
            rospy.logdebug("......relocation successed.....")
        else:
            self.relocation_finish_flag = False
            rospy.logdebug("......relocation failed.....")
            self.ob_exit_recharge_pub.publish(0)#开始匹配充电桩前要发零

        #只有发布了定位完成话题后recharge充电程序才会跑起来
        self.ob_relocation_finish_pub.publish(Bool(self.relocation_finish_flag))
        self.ob_location_status_pub.publish(Bool(self.relocation_finish_flag))
        self.ob_web_tool_runing_pub.publish(False)
        rospy.sleep(1)
        rospy.logdebug("......end_relocation.....")

        # if self.relocation_finish_flag is True:
        #     rospy.loginfo("wait pi starting......")
        #     rospy.sleep(30)
        #     rospy.loginfo("pi start finish......")


        #老化测试程序 test code
        # index_map_goal_id = 0
        # cnt_test = 0
        # rospy.sleep(30)
        # # cnt_test=180
        # while(1):
            
        #     if self.in_recharing ==3 and self.ob_error_status == 0 : 
        #         cnt_test = cnt_test + 1
        #         rospy.loginfo("cnt_test: "+str(cnt_test))
        #         if cnt_test>180:
        #             cnt_test=0
        #             if index_map_goal_id == 0:
        #                 index_map_goal_id = 1
        #                 tempInt32 = Int32()
        #                 tempInt32.data = 3005
        #                 self.ob_map_goal_id_pub.publish(tempInt32)
        #                 rospy.loginfo("......////////////.....")
        #                 rospy.loginfo("......starting auto test to 2001.....")
        #                 rospy.loginfo("......////////////.....")
        #                 # return
        #             elif index_map_goal_id == 1:
        #                 index_map_goal_id = 2
        #                 tempInt32 = Int32()
        #                 tempInt32.data = 3005
        #                 self.ob_map_goal_id_pub.publish(tempInt32)
        #                 rospy.loginfo("......////////////.....")
        #                 rospy.loginfo("......starting auto test to 3001.....")
        #                 rospy.loginfo("......////////////.....")
        #                 # return
        #             elif index_map_goal_id == 2:
        #                 index_map_goal_id = 0
        #                 tempInt32 = Int32()
        #                 tempInt32.data = 3005
        #                 self.ob_map_goal_id_pub.publish(tempInt32)
        #                 rospy.loginfo("......////////////.....")
        #                 rospy.loginfo("......starting auto test to 4001.....")
        #                 rospy.loginfo("......////////////.....")
        #             # return
        #     else:
        #         cnt_test=0
        #     rospy.sleep(1)

    def recharge_handle_cb(self, data):
        self.in_recharing = data.data # 充电过程状态  0:not recharge 1:going to recharge point 2:matching the ir 3:start recharging


    def recharge_status_cb(self, data):
        self.recharge_status = data.data
        # print("self.my_recharge_status:", self.my_recharge_status)

    def ob_error_status_driver_cb(self, data):
        self.ob_error_status = data.data
        

if __name__ == '__main__':

    rospy.init_node('ob_auto_relocation', anonymous=True, log_level=rospy.DEBUG)
    relocation = Relocation()
    rospy.spin()

