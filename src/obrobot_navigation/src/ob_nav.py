#!/usr/bin/env python
#encoding=utf-8


import os
import time

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped,PointStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int32, UInt16, Float32, String, Bool
from sensor_msgs.msg import LaserScan
import math
from std_srvs.srv import Empty
import tf

import dynamic_reconfigure.client
from nav_msgs.srv import GetPlan

from obrobot_navigation.msg import PoseDefine,PoseOperation,CallElevator,ElevatorStatus
from obrobot_navigation.srv import *

import copy


from Robot_mqtt import *
import socket

import json
import getpass
import threading

from map_server.srv import reset_map

from zys_localization.srv import localization_srv



class GoToPose():
    def __init__(self):
        rospy.loginfo("......ob_nav.py is running.....")
        self.count_rec = 0

        self.robot_for_education = True#True #科教版机器人功能标识
        self.robot_for_meal_delivery = False#False #送餐机器人功能标识
        self.robot_for_guest_greeting = False#False #迎宾机器人功能标识

        self.obstaclefront = False
        self.cancelgoalflag = False#取消当前导航标识符
        self.ob_turn_around_flag = False#机器人进行转向标识符
        self.ob_error_status = 0
        self.ob_error_status_last = 0

        self.max_vel_x_normal = 0.8
        self.max_vel_theta_normal = 0.8
        self.xy_goal_tolerance_normal = 0.1
        self.yaw_goal_tolerance_normal = 0.1

        self.max_vel_x_elevator = 0.3
        self.max_vel_theta_elevator = 0.8
        self.xy_goal_tolerance_elevator = 0.1
        self.yaw_goal_tolerance_elevator = 0.1 

        #Arkey 参数是进电梯0.3/0/4,出电梯0.2/0.4
        
        #设置行走速度话题
        rospy.Subscriber("ob_move_speed_setting", Int16, self.ob_move_speed_setting_cb)

        #侦听记录Bag话题
        rospy.Subscriber("ob_record_bag", Int16, self.ob_record_bag_cb)
        self.cnt_ob_record_bag_runing = 0#记录Bag的长度
        self.ob_record_bag_pub=rospy.Publisher("ob_record_bag", Int16, queue_size = 5)  


        self.find_near_pose_nav_path = False#是否找到附近可导航路径点标识符

        self.elevator_up_down = 2 # 1:down  2:up

        #外部配置文件导入
        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
        file_external_cfg = json.load(file)
        file.close()
        self.client_id = file_external_cfg["client_id"]
        rospy.loginfo("client_id:"+str(self.client_id))

        file_Phone_number = file_external_cfg["phone_number"]
        self.Phone_number_call = ','.join(str(i) for i in file_Phone_number)
        rospy.loginfo("Phone_number_call:"+ self.Phone_number_call)

        self.ele = Robot_mqtt(self.client_id)
        self.msg_floor = "0"
        self.msg_room = "0"
        self.msg_location = "0"
        self.msg_status = "0"

        #计时打印Log信息的时间戳
        self.cnt_bagrecord_Beat = 0 

        #### heart beat ####
        self.idleBeat = 3600 # 1 hour
        self.busyBeat = 60 # 1 min
        self.checkBeat = 3
        self.cnt_idleBeat = 0 #count the timer of idlebeat
        self.cnt_busyBeat = 0   #count the timer of busyBeat
        self.firstGetIn = True
        #check every state change for every 3 sec
        self.check = threading.Timer(self.checkBeat, self.beatCheckFn)
        self.check_var_update = threading.Timer(self.checkBeat, self.beatCheckFn_var_update)
        ####################

        #robot working state
        self.idle = 0
        self.delivering = 1
        self.returning = 2
        self.greeting = 3
        self.error = 4


        self.ele_occ_status = 0
        self.floor_occ_status = 0
        self.avoidpoint_occ_status = 0
        self.robot_working_status = 0   #0:空闲状态，1：派送中，2：返回中，3：迎宾中，4：异常中，5：机器人离线

        self.mqtt_connection_cnt = 0

        self.map_goal_id_record = 0 #记录ob_map_goal_id话题下发的可执行导航的导航点的id,用来判断各种情况下的运动
        self.map_goal_pose_record = Pose()#记录目标点坐标


        self.Goal_id_buff = 0
        self.Delivery_TimeBegin = 0 
        self.Delivery_TimeEnd = 0
        self.Delivery_TimeTotal = 0
        self.Delivery_Start_flag = 0#记录派送启动标志位,主要用来判断返回原点后是否要记录派送结束时间和上传派送总时间

        self.currentfloor = 1
        self.exp_floor = 1
        self.elevator_ack_flag = False
        # self.connect_elevator_success = False
        self.elevator_re_outcall_cnt = 0

        #0: nothing 
        #1: nav to the same floor pos 
        #2: nav to elevator wait pose 
        #3: call elevator
        #4: nav to elevator inside pose 
        #9: get in elve fail,nav back to the elev wait point
        #10: get in elve fail,nav to the floor avoid point
        #5: robot turn around
        #6: exchange floor 
        #7: wait for elevatot reach exp floor
        #8: nav to elelvator out pos 
        #11: current floor is occupied,nav to the avoid pos
        #12: get out elve fail,nav back to the elev relocation point
        #15：reach the room pos
        #16: get map goal,robot is get out of the charger
        #20：nav error handle
        #21: waitting error resolve and handle
        self.nav_step = 0 
        self.nav_step_record_befor_error = 0#记录导航出现异常(包括非路线规划的外部异常)前的导航阶段值,用于恢复导航使用
        self.nav_try_cnt = 0
        self.TH_nav_try_cnt = 20#设置路径规划失败退出的次数阈值#test code

        self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作


        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)


        rospy.Subscriber('ob_web_tool_runing', Bool, self.ob_web_tool_runing_cb)
        self.ob_web_tool_runing_Flag=False


	    # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) #初始化move base
        rospy.loginfo("Wait for the action server to come up")
	    # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        self.posemanage = rospy.ServiceProxy('posemanage_server', PoseManage)
        rospy.loginfo("ob_nav.py wait for posemanage_server")
        self.posemanage.wait_for_service()
        rospy.loginfo('found "posemanage_server" service')

        self.getglobalplan = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)

        #动态设置TEB参数
        self.drclient = dynamic_reconfigure.client.Client('/move_base/TebLocalPlannerROS')

        self.tflistener = tf.TransformListener()

        rospy.Subscriber('ob_floor_exchange', Int16, self.ob_floor_exchange_cb)


        #现在改成调用OB重定位话题,只需要给x,y的坐标就可以了
        # initial_pose = Twist()
        # initial_pose.linear.x=data.pose.pose.position.x
        # initial_pose.linear.y=data.pose.pose.position.y
        rospy.Subscriber('ob_localization', Twist, self.ob_localization_cb)

        self.mqtt_comm_fail_cnt = 0 #记录mqtt_comm_fail的次数,用于降低连续请求网络连接的频率,减少流量消耗

        #导航到目标点状态返回：
        #0：用户取消
        #1：成功到达
        #2：路径规划失败且周围附近点也无法到达，进入错误原地等待
        #4：目标点id不存在
        #5：正在导航中，当前请求目标点位置不执行
        #6：机器人异常中，导航取消
        self.ob_nav_finish_pub = rospy.Publisher('ob_nav_finish', Int16, queue_size = 5, latch=True) 


        self.ob_map_goal_info_pub = rospy.Publisher('ob_map_goal_info', Twist, queue_size = 5) 
        self.velpub = rospy.Publisher('cmd_vel', Twist, queue_size = 5) 
        self.infovoicepub = rospy.Publisher('ob_info_voice_play', String, queue_size = 5) 
        self.ob_map_goal_id_outcharge_pub = rospy.Publisher('ob_map_goal_id_outcharge', Int32, queue_size = 5)
        self.ob_robot_working_status_pub = rospy.Publisher('ob_robot_working_status', Int16, queue_size = 5) 
        self.ob_in_moving_pub = rospy.Publisher('ob_in_moving', Int16, queue_size = 5) 
        #发送机器人定位状态话题：定位成功为true,定位失败为false
        self.ob_location_status_pub = rospy.Publisher("ob_location_status", Bool, queue_size = 5)
        self.ob_auto_recharge_pub = rospy.Publisher("ob_auto_recharge", Bool, queue_size = 5)

        # self.ob_elevator_call_pub = rospy.Publisher('ob_elevator_call', CallElevator, queue_size = 5) #呼叫电梯的话题，第一个数字指示上升还是下降, up or down xff x:1 for up; 2 for down
        # self.ob_elevator_keep_control_pub = rospy.Publisher('ob_elevator_keep_control', Int16, queue_size = 5) # 保持或者释放电梯控制权的话题，1：保持电梯控制，0：释放电梯控制
        # self.ob_elevator_status_require_pub = rospy.Publisher('ob_elevator_status_require', Bool, queue_size = 5) # 查询电梯状态的话题，电梯状态需要查询才能上报
        # self.ob_elevator_connect_pub = rospy.Publisher('ob_elevator_connect', Bool, queue_size = 5) # 建立和电梯的连接
        
        #发送机器人网络异常话题，0表示网络恢复正常，1表示机器人自身mqtt连接异常，2表示mqtt连接正常但数据通信异常（OB云服务器通信异常）
        self.ob_mqtt_connect_fail_pub = rospy.Publisher('ob_mqtt_connect_fail', Int16, queue_size = 5, latch=True) # MQTT连接失败

        self.ob_changing_current_map_pub = rospy.Publisher('ob_changing_current_map', Bool, queue_size = 5) # 切换地图中

        self.ob_elevator_info_pub = rospy.Publisher('ob_elevator_info', Twist, queue_size = 5, latch=True) 
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 5)

        #上报当前机器人导航路线被其他机器人占用信息,主要用于给头部界面提示
        self.ob_robot_delivery_status_pub = rospy.Publisher('ob_robot_delivery_status', Int16, queue_size = 5, latch=True) 
        #0:正常无占用   1：电梯被占用   2：楼层被占用   3：避让点被占用

        self.ob_map_goal_id_return_pub = rospy.Publisher('ob_map_goal_id_return', Int16, queue_size = 5)
        
        self.ob_map_goal_id_pub = rospy.Publisher("ob_map_goal_id", Int32, queue_size = 5)

        #发送异常解除话题,0表示异常解除失败,需提示一下,1表示继续任务,2表示返回原点
        self.ob_error_resolve_status_pub = rospy.Publisher("ob_error_resolve_status", Int16, queue_size = 5)

        self.ob_get_robot_time_pub = rospy.Publisher("ob_get_robot_time", Int16, queue_size = 5)


        # 设置搜索附近的目标位置表，第一列是x，正方向沿车头方向，第二列y，正方向为左方向
        self.search_table = [
            [0, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
            [-1, 0, 0],
            [-1, 1, 0],
            [-1, -1, 0],
            [-2, 0, 0],
            [-2, 1, 0],
            [-2, -1, 0]
        ] 
        self.search_radius = 0.3#寻找目标点周围的可到达点半径范围，单位m
        self.search_pose = Pose()#目标点附近的点坐标
        self.search_pose_base = Pose()#目标点坐标
        self.search_pose_base_angle = 0#目标点坐标的角度

        self.goal_sent = False#已经发送目标点给Movebase进行路径规划和导航中标志位
        self.map_goal_moving = 0#机器人正在导航中状态标志位，包括在多次尝试路径规划等状态


        # rospy.Subscriber("ob_relocation_finish", Bool, self.ob_relocation_finish_cb)
        rospy.Subscriber("ob_map_goal_cancel", Bool, self.ob_map_goal_cancel_cb)

        rospy.Subscriber("ob_map_goal_id", Int32, self.ob_map_goal_id_cb)
        rospy.Subscriber('ob_turn_around', Float32, self.ob_turn_around_cb)
        rospy.Subscriber('ob_error_status_driver', Int16, self.ob_error_status_driver_cb) 
        rospy.Subscriber('ob_obstaclefront', Bool, self.ob_obstaclefront_cb) 
        rospy.Subscriber("recharge_handle", Int16, self.recharge_handle_cb)
        # rospy.Subscriber('ob_elevator_status', ElevatorStatus, self.ob_elevator_status_cb) 
        rospy.Subscriber("ob_current_floor", Int16, self.current_floor_cb)
        self.ob_current_floor_pub = rospy.Publisher("ob_current_floor", Int16, queue_size = 5, latch=True)
        # rospy.Subscriber('ob_connect_elevator_status', Bool, self.ob_connect_elevator_status_cb)
        rospy.Subscriber("ob_locker_error_status_report", Int16, self.locker_error_status_report_cb)
        #侦听派送超时返回话题
        rospy.Subscriber("ob_delivery_overtime", Int16, self.ob_delivery_overtime_cb)

        #在机器人异常情况下接收外部操作,用于手动解除异常
        #0:不做任何处理
        #1:继续当前任务
        #2:返回原点
        rospy.Subscriber("ob_error_resolve_method", Int16, self.ob_error_resolve_method_cb)
        #发送解除引起机器人卸力的异常,包括急停\过流\导航失败定位失败等
        self.ob_error_resolve_driver_control_pub = rospy.Publisher('ob_error_resolve_driver_control', Int16, queue_size = 5) 
        self.current_pose = PoseWithCovarianceStamped()
        self.in_recharing = 0
        # rospy.sleep(3)


    def ob_move_speed_setting_cb(self, data):
        if data.data>=300 and data.data<=1000:
            self.max_vel_x_normal = data.data/1000.0

            #设定机器人正常的行走TEB参数
            self.drclient.update_configuration({'max_vel_x': self.max_vel_x_normal})


    #录制BAG包功能函数,输入0表示停止录制,输入大于0数字表示加长录制时间,单位是20s
    def ob_record_bag_cb(self, data):
        
        if self.ob_web_tool_runing_Flag==False:
            
            temp_data = self.cnt_ob_record_bag_runing
            if data.data>0:#
                self.cnt_ob_record_bag_runing +=data.data # 传入参数是录包的时间
            else:
                self.cnt_ob_record_bag_runing=0
            if temp_data==0 and self.cnt_ob_record_bag_runing!=0:
                t1 = threading.Thread(target=self.get_bagfile)
                t1.start()

            rospy.loginfo('get ob_record_bag_cb!!!!'+ str(self.cnt_ob_record_bag_runing))

    def get_bagfile(self):
        t1 = threading.Thread(target= lambda: os.system("rosbag record -a -o /home/ob/.ob/bag/"))
        t1.start()
        cnt_record_period = 0 
        # while(self.cnt_ob_record_bag_runing>0 or (self.cnt_ob_record_bag_runing<=0 and cnt_record_period!=0)):
        cnt_ob_record_bag_runing=copy.deepcopy(self.cnt_ob_record_bag_runing)
        while(cnt_ob_record_bag_runing>0):
            time.sleep(5)
            cnt_ob_record_bag_runing=copy.deepcopy(self.cnt_ob_record_bag_runing)
            cnt_record_period+=5
            if cnt_record_period>=20:#录制时间周期20s
                cnt_record_period=0
                cnt_ob_record_bag_runing-=1
                self.cnt_ob_record_bag_runing=copy.deepcopy(cnt_ob_record_bag_runing)         

        os.system("echo 'obrobot'|sudo -S bash /home/ob/.ob/autostart/ros_bag_close.sh")
        t1.join()
        self.cnt_ob_record_bag_runing=0

    def ob_error_status_driver_cb(self, data):
        self.ob_error_status = data.data
        
        #发送机器人状态给到mqtt程序
        self.ele.getRobotStateFromRobot(self.ob_error_status,self.nav_step)

        #异常情况下发送短信提示
        if self.ob_error_status == 0 and self.ob_error_status_last != 0:  
            rospy.loginfo('error status clear !!!:'+ str(self.ob_error_status_last))
            if self.in_recharing == 3:   
                if self.Delivery_Start_flag!=0:
                    self.Delivery_Start_flag=0
                    rospy.loginfo("Delivery Fail, robot in charge and error clear!!!!")
                self.nav_step = 0
                #坐上充电桩，清除当前所有占用
                self.ele_occ_status = 0
                self.floor_occ_status = 0
                self.avoidpoint_occ_status = 0
                self.robot_working_status = 0
                self.ob_robot_working_status_pub.publish(self.robot_working_status)
                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)

        elif self.ob_error_status != 0 and self.ob_error_status_last == 0:
            rospy.loginfo('get error status!!!:'+ str(self.ob_error_status))
            
            # self.ob_record_bag_pub.publish(1)#启动一次bag包记录

            #发送机器人调试信息话题
            self.ele.pubDebuginfo(self.ob_error_status,self.nav_step,self.Goal_id_buff,self.robot_working_status)

            #记录机器人异常前的导航阶段值用于恢复任务执行
            if self.nav_step!=20 and self.nav_step!=21:
                self.nav_step_record_befor_error = self.nav_step
                rospy.loginfo('nav_step_record_befor_error:'+ str(self.nav_step_record_befor_error))
            #发送异常必须首先取消导航,因为如果不取消导航就无法退出goto函数,mainloop就无法继续进行
            #取消当前目标点导航
            if self.goal_sent is True:
                self.cancelgoalflag = True
                self.move_base.cancel_goal()
                rospy.loginfo("robot error ,cancel current goal")
            self.map_goal_moving = 0
            self.ob_in_moving_pub.publish(self.map_goal_moving)

            self.nav_step = 20  #20：nav error handle
            self.nav_try_cnt = 0
            self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
            rospy.loginfo("nav_step: "+str(self.nav_step))

            #设定机器人正常的行走TEB参数
            self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_normal, 'yaw_goal_tolerance': self.yaw_goal_tolerance_normal , 'max_vel_x': self.max_vel_x_normal , 'max_vel_theta': self.max_vel_theta_normal})
            
            #更新机器人工作状态并上报   
            self.robot_working_status = 4
            self.ob_robot_working_status_pub.publish(self.robot_working_status)
            self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)

            if self.nav_step_record_befor_error != 0 :
                
                if self.nav_step_record_befor_error == 1 or self.nav_step_record_befor_error == 2:
                    #路上
                    self.msg_floor = str(self.currentfloor).rjust(2,'0')
                    self.msg_room = str(self.Goal_id_buff+self.nav_step_record_befor_error*1000).rjust(5,'0')
                    self.msg_location = str(3).rjust(2,'0')
                    self.msg_status = str(1).rjust(2,'0')
                    self.ele.send_message(self.Phone_number_call, self.msg_floor,self.msg_room,self.msg_location,self.msg_status)

                elif self.nav_step_record_befor_error == 3 or self.nav_step_record_befor_error == 4 or self.nav_step_record_befor_error == 9 or self.nav_step_record_befor_error == 10:
                    #电梯门口

                    # tcp释放电梯控制权
                    self.ele.elevatorCtrl(fOpenCtrl='0')
                    rospy.loginfo("---nav error when get in the elve,release the opendoor key---")

                    self.msg_floor = str(self.currentfloor).rjust(2,'0')
                    self.msg_room = str(self.Goal_id_buff+self.nav_step_record_befor_error*1000).rjust(5,'0')
                    self.msg_location = str(2).rjust(2,'0')
                    self.msg_status = str(1).rjust(2,'0')
                    self.ele.send_message(self.Phone_number_call, self.msg_floor,self.msg_room,self.msg_location,self.msg_status)
   
                elif self.nav_step_record_befor_error == 5 or self.nav_step_record_befor_error == 6  or self.nav_step_record_befor_error == 7 or self.nav_step_record_befor_error == 8  or self.nav_step_record_befor_error == 11 or self.nav_step_record_befor_error == 12:
                    #电梯中

                    # tcp释放电梯控制权
                    self.ele.elevatorCtrl(fOpenCtrl='0')
                    rospy.loginfo("---nav error when get out the elve,release the opendoor key---")

                    self.msg_floor = str(self.currentfloor).rjust(2,'0')
                    self.msg_room = str(self.Goal_id_buff+self.nav_step_record_befor_error*1000).rjust(5,'0')
                    self.msg_location = str(1).rjust(2,'0')
                    self.msg_status = str(1).rjust(2,'0')
                    self.ele.send_message(self.Phone_number_call, self.msg_floor,self.msg_room,self.msg_location,self.msg_status)
                           
                elif self.nav_step_record_befor_error == 15 :
                    #房间门口
                    self.msg_floor = str(self.currentfloor).rjust(2,'0')
                    self.msg_room = str(self.Goal_id_buff+self.nav_step_record_befor_error*1000).rjust(5,'0')
                    self.msg_location = str(4).rjust(2,'0')
                    self.msg_status = str(1).rjust(2,'0')
                    self.ele.send_message(self.Phone_number_call, self.msg_floor,self.msg_room,self.msg_location,self.msg_status)
            
            self.nav_step = 21   #21: waitting error resolve and handle
            self.nav_try_cnt = 0
            self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
            rospy.loginfo("nav_step: "+str(self.nav_step))

            rospy.sleep(2)#driver程序释放底盘卸力报错,导致error发送错误话题延迟,需要等待

        self.ob_error_status_last = self.ob_error_status


    #在机器人异常情况下接收外部操作,用于手动解除异常
    def ob_error_resolve_method_cb(self, data):
        rospy.loginfo('ob_error_resolve_method_cb:'+ str(data.data))
        
        if True:
            if data.data==0:
                #发送异常解除话题,1表示继续任务,2表示返回原点
                self.ob_error_resolve_status_pub.publish(0)
                rospy.loginfo('ob_error_resolve_method is zero, nothing to do ') 
            else:
                #要执行恢复机器人操作,先判断是否有过流保护或者充电异常保护或者急停按键被按下等导致电机失能的异常,先软件清除异常
                self.ob_error_resolve_driver_control_pub.publish(1)
                rospy.sleep(3)#延时等待异常标志位更新清零

                if self.ob_error_status!=0:
                    #异常没有解除,不能继续进行下一步操作
                    rospy.loginfo('error need to be resolve firse!!!:'+ str(self.ob_error_status))                
                    #发送异常解除话题,1表示继续任务,2表示返回原点
                    self.ob_error_resolve_status_pub.publish(0)
                else:
                    
                    # #继续执行解除异常操作之前先进行一次重定位

                    # #启动一次YS重定位操作
                    # #先求出方位角
                    # self.tflistener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(60))
                    # (_trans, _rot) = self.tflistener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
                    # _euler1 = tf.transformations.euler_from_quaternion(_rot)
                    # self.yaw_cur = _euler1[2]
                    # self.current_pose.pose.pose.position.x = _trans[0]
                    # self.current_pose.pose.pose.position.y = _trans[1]
                    # self.current_pose.pose.pose.position.z = _trans[2]
                    # self.current_pose.pose.pose.orientation.x = _rot[0]    # Makes the origin quaternion valid.
                    # self.current_pose.pose.pose.orientation.y = _rot[1]    # Makes the origin quaternion valid.
                    # self.current_pose.pose.pose.orientation.z = _rot[2]    # Makes the origin quaternion valid.
                    # self.current_pose.pose.pose.orientation.w = _rot[3]    # Makes the origin quaternion valid.

                    # self.yaw_cur = self.yaw_cur/math.pi*180 #弧度转角度
            
                    # self.ob_localization = rospy.ServiceProxy('initial_pose_one', localization_srv)
                    # #rospy.loginfo("ob_error_resolve wait for ob_localization")
                    # self.ob_localization.wait_for_service()
                    # #rospy.loginfo('found "ob_localization" service')
                    # self.ob_changing_current_map_pub.publish(True)
                    # # rospy.sleep(0.3)
                    # #rospy.loginfo('start localization!!')
                    # tempresult = self.ob_localization(self.current_pose.pose.pose.position.x,self.current_pose.pose.pose.position.y,self.yaw_cur)
                    # #if tempresult.result is True:
                    #     #rospy.loginfo("ob_localization finish,score："+ str(tempresult.score))
                    # self.ob_changing_current_map_pub.publish(False)
                    
                    if data.data==1:
                        #发送异常解除话题,1表示继续任务,2表示返回原点
                        self.ob_error_resolve_status_pub.publish(1)
                        #继续当前任务
                        if self.nav_step_record_befor_error==0:
                            #当前任务为0,不做任何操作
                            rospy.loginfo('robot is no mission to continue!!!!'+ str(self.ob_error_status))               
                            #返回原点
                            rospy.loginfo('robot return to the orign pos!!!!')                     
                            tempInt32 = Int32()
                            tempInt32.data = 100002
                            self.ob_map_goal_id_pub.publish(tempInt32)
                        else:
                            
                            #先恢复机器人工作状态
                            if self.map_goal_id_record<100000:#派送到房间点
                                self.robot_working_status = 1
                                self.ob_robot_working_status_pub.publish(self.robot_working_status)
                                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
                            elif self.map_goal_id_record==100002:#导航到原点，表示派送完成返回过程
                                self.robot_working_status = 2
                                self.ob_robot_working_status_pub.publish(self.robot_working_status)
                                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
                                                            
                            if self.nav_step_record_befor_error == 16: #16: get map goal,robot is get out of the charger 
                                #重新启动订单派送
                                rospy.loginfo('restart the map goal send!!!!')                     
                                tempInt32 = Int32()
                                tempInt32.data = self.map_goal_id_record
                                self.ob_map_goal_id_pub.publish(tempInt32)
                                self.nav_step = 0
                            elif self.nav_step_record_befor_error == 15: #15：reach the room pos
                                if self.robot_for_meal_delivery == False:#False #送餐机器人功能标识
                                    #返回原点
                                    rospy.loginfo('robot return to the orign pos!!!!')                     
                                    tempInt32 = Int32()
                                    tempInt32.data = 100002
                                    self.ob_map_goal_id_pub.publish(tempInt32)
                                self.nav_step = 0
                            elif self.nav_step_record_befor_error == 4 or self.nav_step_record_befor_error == 9: 
                                #4: nav to elevator inside pose
                                #9: get in elve fail,nav back to the elev wait point
                                #机器人在进入电梯出现异常,将状态返回到呼叫电梯阶段
                                self.nav_step = 3#3: call elevator                              
                            elif self.nav_step_record_befor_error == 8 or self.nav_step_record_befor_error == 12 : 
                                #8: nav to elelvator out pos 
                                #12: get out elve fail,nav back to the elev relocation point
                                #机器人在出电梯出现异常,将状态返回到呼叫电梯到达目标楼层阶段
                                self.nav_step = 7#7: wait for elevatot reach exp floor
                            else:
                                self.nav_step = self.nav_step_record_befor_error         

                            self.nav_try_cnt = 0
                            self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作  
                            rospy.loginfo("nav_step: "+str(self.nav_step)) 
                            rospy.loginfo('mission continue!!!!')
                    elif data.data==2:
                        #发送异常解除话题,1表示继续任务,2表示返回原点
                        self.ob_error_resolve_status_pub.publish(2)

                        #返回原点
                        rospy.loginfo('robot return to the orign pos!!!!')  

                        if (self.nav_step_record_befor_error == 5) or (self.nav_step_record_befor_error == 6): 
                            #此时机器人在电梯里,还没到出电梯的阶段,需要做的是先把目标点设置为原点,然后重新呼叫一次电梯预约
                            poseoperation=PoseOperation()
                            poseoperation.opt   = "select"
                            poseoperation.id    = 100002
                            resp = self.posemanage(poseoperation)
                            if "FAIL" in resp.status:
                                rospy.loginfo("cannot find the pose:"+str(poseoperation.id))     
                            
                            self.exp_floor = resp.posedefine.floor
                            rospy.loginfo("exp_floor:"+str(self.exp_floor))
                            self.map_goal_id_record=resp.posedefine.id #记录ob_map_goal_id话题下发的可执行导航的导航点的id,用来判断各种情况下的运动
                            self.map_goal_pose_record = resp.posedefine.pose#记录目标点坐标

                            self.nav_step = self.nav_step_record_befor_error
                            self.nav_try_cnt = 0
                            self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作  
                            rospy.loginfo("nav_step: "+str(self.nav_step)) 

                        elif (self.nav_step_record_befor_error == 7) or (self.nav_step_record_befor_error == 8) or (self.nav_step_record_befor_error == 12): 
                            #此时机器人在电梯里,还没到出电梯的阶段,需要做的是先把目标点设置为原点,目标楼层也设置为1楼,然后切换一次地图
                            poseoperation=PoseOperation()
                            poseoperation.opt   = "select"
                            poseoperation.id    = 100002
                            resp = self.posemanage(poseoperation)
                            if "FAIL" in resp.status:
                                rospy.loginfo("cannot find the pose:"+str(poseoperation.id))     
                            
                            self.exp_floor = resp.posedefine.floor
                            rospy.loginfo("exp_floor:"+str(self.exp_floor))
                            self.map_goal_id_record=resp.posedefine.id #记录ob_map_goal_id话题下发的可执行导航的导航点的id,用来判断各种情况下的运动
                            self.map_goal_pose_record = resp.posedefine.pose#记录目标点坐标

                            # 更改当前地图并且设置初始点
                            self.ob_changing_current_map_pub.publish(True)
                            rospy.loginfo("-------------change floor map-----------------------")
                            self.reset_map = rospy.ServiceProxy('reset_map', reset_map)
                            floor_test_src = "/home/ob/.ob/map/OB_floor_" + str(self.exp_floor) + ".yaml"  #目标楼层
                            tempresult = self.reset_map(floor_test_src, "map")
                            floor_test_src = "/home/ob/.ob/map/OB_floor_location_" + str(self.exp_floor) + ".yaml"  #目标楼层
                            tempresult = self.reset_map(floor_test_src, "map_amcl")
                            if tempresult.result is True:
                                self.currentfloor = int(self.exp_floor)
                                self.ob_current_floor_pub.publish(self.currentfloor)
                                rospy.loginfo("self.currentfloor:"+str(self.currentfloor))
                            rospy.sleep(3)
                            #先在电梯内做个初始的重定位,等会电梯到了目标楼层后出来前再做一次YS重定位
                            poseoperation=PoseOperation()
                            poseoperation.opt   = "select"
                            poseoperation.id    = 100004 + self.exp_floor * 1000
                            resp = self.posemanage(poseoperation)
                            if "FAIL" in resp.status:
                                rospy.loginfo("cannot find the pose:"+str(poseoperation.id))

                            self.ob_changing_current_map_pub.publish(True)
                            rospy.sleep(1)
                            # rospy.loginfo('start localization!!')
                            origin_pose = PoseWithCovarianceStamped()
                            origin_pose.header.frame_id = "/map"
                            origin_pose.pose.pose.position.x = resp.posedefine.pose.position.x
                            origin_pose.pose.pose.position.y = resp.posedefine.pose.position.y
                            origin_pose.pose.pose.position.z = resp.posedefine.pose.position.z
                            origin_pose.pose.pose.orientation.x = resp.posedefine.pose.orientation.x    
                            origin_pose.pose.pose.orientation.y = resp.posedefine.pose.orientation.y    
                            origin_pose.pose.pose.orientation.z = resp.posedefine.pose.orientation.z    
                            origin_pose.pose.pose.orientation.w = resp.posedefine.pose.orientation.w    
                            origin_pose.pose.covariance[0] = 0.02
                            origin_pose.pose.covariance[7] = 0.02
                            # origin_pose.pose.covariance[14] = 1.0
                            # origin_pose.pose.covariance[21] = 1.0
                            # origin_pose.pose.covariance[28] = 1.0
                            origin_pose.pose.covariance[35] = 0.02
                            self.initialpose_pub.publish(origin_pose)
                            rospy.sleep(1)
                            self.ob_changing_current_map_pub.publish(False)

                            # rospy.loginfo("getout elevator initialpose ")

                            self.nav_step = 12
                            self.nav_try_cnt = 0
                            self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作  
                            rospy.loginfo("nav_step: "+str(self.nav_step)) 

                        else:
                            tempInt32 = Int32()
                            if self.robot_for_guest_greeting == True:#False #迎宾机器人功能标识
                                tempInt32.data = 100003
                            else:
                                tempInt32.data = 100002
                            self.ob_map_goal_id_pub.publish(tempInt32)
                            rospy.loginfo('return to orgin pos!!!!')
        # else:
        #     #发送异常解除话题,1表示继续任务,2表示返回原点
        #     self.ob_error_resolve_status_pub.publish(0)
        #     rospy.loginfo('no roboto error, nothing to do ')       
    
    
    #启动一次YS重定位操作
    def ob_localization_cb(self, data):
        rospy.loginfo('Receive ob_localization_cb')

        self.ob_changing_current_map_pub.publish(True)
        # rospy.sleep(0.3)
        # rospy.loginfo('start localization!!')
        self.ob_localization = rospy.ServiceProxy('initial_pose_one', localization_srv)
        tempresult = self.ob_localization(data.linear.x,data.linear.y,data.linear.z)
        # if tempresult.result is True:
            # rospy.loginfo("ob_localization finish,score："+ str(tempresult.score))
        self.ob_changing_current_map_pub.publish(False)



    #启动一次切换地图操作
    def ob_floor_exchange_cb(self, data):
        
        rospy.loginfo('Receive map switch:'+ str(data.data))
        if (self.robot_for_education == False and self.robot_for_meal_delivery == False) or data.data==self.currentfloor: #科教版机器人功能标识

            self.ob_changing_current_map_pub.publish(True)
            self.reset_map = rospy.ServiceProxy('reset_map', reset_map)
            floor_test_src = "/home/ob/.ob/map/OB_floor_" + str(data.data) + ".yaml"  #目标楼层
            tempresult = self.reset_map(floor_test_src, "map")
            floor_test_src = "/home/ob/.ob/map/OB_floor_location_" + str(data.data) + ".yaml"  #目标楼层
            tempresult = self.reset_map(floor_test_src, "map_amcl")
            if tempresult.result is True:
                self.currentfloor = int(data.data)
                self.ob_current_floor_pub.publish(self.currentfloor)
                rospy.loginfo("self.currentfloor:"+str(self.currentfloor))
            rospy.sleep(3)
            self.ob_changing_current_map_pub.publish(False)

    def ob_delivery_overtime_cb(self, data):
        if data.data == 1:

            # #++++++++++++++++++++++++短信发送 begin++++++++++++++++++++++++           
            # self.msg_floor = str(self.currentfloor).rjust(2,'0')
            # self.msg_room = str(self.Goal_id_buff+99000).rjust(5,'0')
            # self.msg_location = str(4).rjust(2,'0')
            # self.msg_status = str(3).rjust(2,'0')
            # self.ele.send_message('18298119955,13422345195,15800164782', self.msg_floor,self.msg_room,self.msg_location,self.msg_status)
            # #++++++++++++++++++++++++短信发送 end++++++++++++++++++++++++
            # rospy.loginfo("#############send  message of locker_error_status_report")

            #发送机器人调试信息话题
            self.ele.pubDebuginfo(self.ob_error_status,99,self.Goal_id_buff,self.robot_working_status)


    def locker_error_status_report_cb(self, data):
        if data.data == 1:

            #++++++++++++++++++++++++短信发送 begin++++++++++++++++++++++++           
            self.msg_floor = str(self.currentfloor).rjust(2,'0')
            self.msg_room = str(self.Goal_id_buff).rjust(5,'0')
            self.msg_location = str(4).rjust(2,'0')
            self.msg_status = str(3).rjust(2,'0')
            self.ele.send_message(self.Phone_number_call, self.msg_floor,self.msg_room,self.msg_location,self.msg_status)
            #++++++++++++++++++++++++短信发送 end++++++++++++++++++++++++
            # rospy.loginfo("#############send  message of locker_error_status_report")

    # 获取当前的楼层
    def current_floor_cb(self, data):
        self.currentfloor = int(data.data)
        rospy.loginfo("self.currentfloor:"+str(self.currentfloor))


    # 电梯实时状态回调
    # def ob_elevator_status_cb(self, data):
    #     self.elevator_status.CurrentFloor_num = data.CurrentFloor_num
    #     self.elevator_status.FrontDoor = data.FrontDoor


    # # 连接电梯是否成功状态回调
    # def ob_connect_elevator_status_cb(self, data):
    #     self.connect_elevator_success = data.data
        
    def ob_web_tool_runing_cb(self, data):
        self.ob_web_tool_runing_Flag = data.data


    def recharge_handle_cb(self, data):
        self.in_recharing = data.data # 充电过程状态  0:not recharge 1:going to recharge point 2:matching the ir 3:start recharging
        if self.in_recharing == 3:   
            if self.Delivery_Start_flag!=0:
                self.Delivery_Start_flag=0
                rospy.loginfo("Delivery Fail, robot in charge!!!!")
            self.nav_step = 0        #0: nothing 
            #坐上充电桩，清除当前所有占用
            self.ele_occ_status = 0
            self.floor_occ_status = 0
            self.avoidpoint_occ_status = 0
            self.robot_working_status = 0
            self.ob_robot_working_status_pub.publish(self.robot_working_status)
            self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
            rospy.loginfo("in_recharing... clear all of robot status...")


    # 获取激光雷达探测的机器人前方障碍物信息
    def ob_obstaclefront_cb(self, data):
        self.obstaclefront = data.data


    def goto(self, goal):
        # Send a goal
        # return 1: success
        # return 2: cancel
        # return 3: failure

        self.goal_sent = True
	    
        #启动MoveBase路径规划和导航之前先清一下costmap,
        clearcostmaps_srv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        tempresult = clearcostmaps_srv()
        # rospy.loginfo("start navigation,clear costmap")
        rospy.sleep(0.1)   #210515 1s ->0.1s


        retry_cnt = 0
        while(retry_cnt < 1):#只尝试1次路径规划和导航
            retry_cnt += 1

            self.move_base.send_goal(goal) #给move base发送目标位置
            # 设置了到目标点10min超时
            success = self.move_base.wait_for_result(rospy.Duration(600)) #一直在此处等待move base的结果
            state = self.move_base.get_state() # 获取目标点的结果状态
            result = 0
            #rospy.loginfo("success:" + str(success) + " ; " + "state:" + str(state))
            if state == GoalStatus.SUCCEEDED:
                # 成功达到
                #rospy.loginfo("reach success")
                result = 1
                break
            elif self.cancelgoalflag is True:
                # 用户取消
                result = 2
                break
            elif ((state == GoalStatus.ABORTED) or (success == False)):
                # 中断，可能收到新的目标位置，或者失败
                result = 3
            else:
                # 其他情况
                result = 0
        
        self.cancelgoalflag = False
        self.goal_sent = False
        return result

    # 接收到取消目标点的命令
    def ob_map_goal_cancel_cb(self, data):
        if data.data is True :
            if self.map_goal_moving != 0:
                #在机器人处于导航状态下处理新导航需求,
                if self.in_recharing == 0 and (self.nav_step == 0 or self.nav_step == 1 or self.nav_step == 2 or self.nav_step == 3 or self.nav_step == 10 or self.nav_step == 11 or self.nav_step == 15 or self.nav_step == 16):
                    #如果机器人没有处于充电状态且在不在进出电梯或电梯内则可以停止导航
                    if self.goal_sent is True:
                        self.cancelgoalflag = True
                        self.move_base.cancel_goal()
                        rospy.loginfo("robot cancel goal success")
                    self.map_goal_moving=0
                    self.ob_in_moving_pub.publish(self.map_goal_moving)
                    self.ob_nav_finish_pub.publish(0)
                    self.nav_step = 0
                else:
                    rospy.loginfo("robot cancel goal fail,in spc condition")     
            else:
                rospy.loginfo("robot cancel goal fail,no in moving status")                       


    # 通过id设置目标位置
    def ob_map_goal_id_cb(self, data):
        
        # 判断目标位置id是否存在
        poseoperation=PoseOperation()
        poseoperation.opt   = "select"
        poseoperation.id    = data.data
        rospy.loginfo("get ob_map_goal_id:"+str(poseoperation.id))

        try:
            resp = self.posemanage(poseoperation)
            if "SUCCESS" in resp.status:
                
                if self.robot_for_education == True or self.robot_for_meal_delivery == True: #科教版机器人功能标识 

                    if resp.posedefine.floor != self.currentfloor:
                        self.ob_nav_finish_pub.publish(4) # 如果id不存在，则nav_finish返回4
                        self.ob_map_goal_id_return_pub.publish(0)
                        rospy.loginfo("cannot got to different floor goal!!!! ")
                        return

                #返回当前id是否存在
                self.ob_map_goal_id_return_pub.publish(1)

                if self.ob_error_status != 0:
                    self.ob_nav_finish_pub.publish(6)
                    rospy.loginfo("robot errror, navigation is cancle")
                else:

                    if self.map_goal_moving != 0:
                        #在机器人处于导航状态下处理新导航需求,
                        if self.in_recharing == 0 and (self.nav_step == 0 or self.nav_step == 1 or self.nav_step == 2 or self.nav_step == 3 or self.nav_step == 10 or self.nav_step == 11 or self.nav_step == 15 or self.nav_step == 16):
                            #如果机器人没有处于充电状态且在不在进出电梯或电梯内则可以更改新的导航点
                            #先取消当前目标点导航，然后启动新的目标点导航
                            if self.goal_sent is True:
                                self.cancelgoalflag = True
                                self.move_base.cancel_goal()
                            rospy.loginfo("robot get new gola ,cancel current goal")                     
                            self.map_goal_moving = 0
                            self.ob_in_moving_pub.publish(self.map_goal_moving)
                            self.ob_nav_finish_pub.publish(0)
                            self.nav_step = 0 
                        else:
                            self.ob_nav_finish_pub.publish(5)
                            rospy.loginfo("map_goal_moving, do not accacpt another pos navigation!") 

                    if self.map_goal_moving == 0:
                        
                        #如果正在充电则先取消自动充电然后再导航到目标点
                        if self.in_recharing != 0 and data.data!=100001:
                            if self.in_recharing == 3:
                                self.infovoicepub.publish("ob_start_to_deliver")
                                self.ob_auto_recharge_pub.publish(False)#发送取消自动充电功能
                                self.ob_map_goal_id_outcharge_pub.publish(data.data)#发送退出导航后的导航目标点
                            
                                #此时已经算是正式开始订单派送了,所以要记录相关信息
                                self.map_goal_id_record=resp.posedefine.id #记录ob_map_goal_id话题下发的可执行导航的导航点的id,用来判断各种情况下的运动
                                self.map_goal_pose_record = resp.posedefine.pose#记录目标点坐标
                                self.nav_step = 16    #16: get map goal,robot is get out of the charger 
                                self.nav_try_cnt = 0
                                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                                rospy.loginfo("nav_step: "+str(self.nav_step))


                            else:
                                self.ob_nav_finish_pub.publish(5)
                        else:
                            #机器人接收并执行目标点导航指令
                            self.exp_floor = resp.posedefine.floor
                            rospy.loginfo("exp_floor:"+str(self.exp_floor))

                            self.map_goal_id_record=resp.posedefine.id #记录ob_map_goal_id话题下发的可执行导航的导航点的id,用来判断各种情况下的运动

                            if data.data<100000:#派送到房间点
                                
                                #记录目标点ID
                                self.Goal_id_buff = data.data
                                #++++++++++++++++++++++++派送云端统计 begin++++++++++++++++++++++++
                                self.Delivery_TimeBegin = rospy.get_rostime()
                                self.msg_room = str(self.Goal_id_buff).rjust(5,'0')
                                self.ele.startDeliverReport(self.msg_room,self.Delivery_TimeBegin.secs)
                                self.Delivery_Start_flag = 1#记录派送启动标志位,主要用来判断返回原点后是否要记录派送结束时间和上传派送总时间

                                #当前为派送出发，清除当前所有占用,机器人进入工作状态
                                self.ele_occ_status = 0
                                self.floor_occ_status = 0
                                self.avoidpoint_occ_status = 0
                                self.robot_working_status = 1
                                self.ob_robot_working_status_pub.publish(self.robot_working_status)
                                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)

                            elif data.data==100002:#导航到原点，表示派送完成返回过程
                                self.robot_working_status = 2
                                self.ob_robot_working_status_pub.publish(self.robot_working_status)
                                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)

                            #根据是否跨楼层进行不同的信息判断
                            if self.exp_floor == self.currentfloor:
                                rospy.loginfo("receive goal in the same floor")
                                   
                                if self.map_goal_moving==0:
                                    self.map_goal_moving = 1
                                    self.ob_in_moving_pub.publish(self.map_goal_moving)
                                # 已经在相同的楼层，直接导航到目标房间
                                self.map_goal_pose_record = resp.posedefine.pose#记录目标点坐标
                                self.nav_step = 1    #1: nav to the same floor pos 
                                self.nav_try_cnt = 0
                                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                                rospy.loginfo("nav_step: "+str(self.nav_step))
                            else:
                                #跨楼层，查询等待当前楼层占用清除及目标楼层避让点清除及电梯被占用清除
                                rospy.loginfo("receive different floor goal")

                                if self.map_goal_moving==0:
                                    self.map_goal_moving = 1
                                    self.ob_in_moving_pub.publish(self.map_goal_moving)
                                # 跨楼层,先进行乘梯操作 
                                self.map_goal_pose_record = resp.posedefine.pose#记录目标点坐标
                                if self.exp_floor > self.currentfloor:
                                    self.elevator_up_down = 2
                                else:
                                    self.elevator_up_down = 1
                                self.nav_step = 2        #2: nav to elevator wait pose
                                self.nav_try_cnt = 0
                                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                                rospy.loginfo("nav_step: "+str(self.nav_step))

                            #发布导航信息话题
                            map_goal_info = Twist()
                            map_goal_info.linear.x=resp.posedefine.id
                            map_goal_info.linear.y=resp.posedefine.floor#这里改成楼层信息
                            self.ob_map_goal_info_pub.publish(map_goal_info)
            else:
                # rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.ob_nav_finish_pub.publish(4) # 如果id不存在，则nav_finish返回4
                self.ob_map_goal_id_return_pub.publish(0)
                rospy.loginfo("navigation goal is not exit")
                return

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return

    def Robot_Connect_MQTT(self):
        #TCP连接电梯++++++++++++++++++++++++
        if self.ele.connectionState is False:
            while(not rospy.is_shutdown()):
                rospy.loginfo("---try to connect elevator---")
                try:
                    self.mqtt_connection_cnt = self.mqtt_connection_cnt + 1
                    # print('self.mqtt_connection_cnt:',self.mqtt_connection_cnt)
                    self.ele.connectToServer()
                except socket.errno:
                    print('******socket error')
                except IOError, e:
                    if e.errno == 101:
                        print('******Network is unreachable')
                    elif e.errno == 111:
                        print('******connection refused')
                    elif e.errno == 113:
                        print('no route to host')


                rospy.sleep(1)
                
                if self.ele.connectionState is True:
                    rospy.sleep(0.1)
                    break
                if self.mqtt_connection_cnt >= 5:
                    self.mqtt_connection_cnt=5
                    self.ob_mqtt_connect_fail_pub.publish(1)
                    rospy.loginfo("flag_mqtt_comm_fail_pub:1")
                
        if self.ele.connectionState is True and self.mqtt_connection_cnt!=0 :      
            self.mqtt_connection_cnt = 0    
            # rospy.loginfo("---connect elevator success---")
            self.ob_mqtt_connect_fail_pub.publish(0)
            rospy.loginfo("flag_mqtt_comm_fail_pub:0")

    def startHeartBeat(self):
        self.check = threading.Timer(self.checkBeat, self.beatCheckFn)
        self.check.start()

    def startHeartBeat_var_update(self):        
        self.check_var_update = threading.Timer(self.checkBeat, self.beatCheckFn_var_update)
        self.check_var_update.start()

    def cancelHeartBeat(self):
        print('before cancel CheckThread:'+str(self.check))
        if self.check.is_alive() is True:
            self.check.cancel()
        if self.idlethread.is_alive() is True:
            self.idlethread.cancel()
        if self.busythread.is_alive() is True:
            self.busythread.cancel()

        print('cancelCheckThread:'+str(self.check))
        # +' idleThread:'+self.idlethread+' busythread:'+self.busythread)

        #心跳中断响应函数，3秒进来一次
    def beatCheckFn(self):
        
        # rospy.loginfo('check beat fn is runing')
        self.Robot_Connect_MQTT()
        if self.ele.connectionState is True:
            if self.firstGetIn == True:
                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
                self.firstGetIn = False
            if self.robot_working_status == 0: #idle state
                self.cnt_idleBeat += self.checkBeat
                self.cnt_busyBeat = 0
                if self.cnt_idleBeat>=self.idleBeat:
                    self.cnt_idleBeat-=self.idleBeat
                    rospy.loginfo('send robot heartbeat in idle status')
                    self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
            else:
                self.cnt_busyBeat += self.checkBeat
                self.cnt_idleBeat = 0
                if self.cnt_busyBeat>=self.busyBeat:
                    self.cnt_busyBeat-=self.busyBeat
                    rospy.loginfo('send robot heartbeat in no idle status')
                    self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)                   

        #repeat beatCheckFn
        # rospy.loginfo('check beat fn repeat')
        self.startHeartBeat()

    def beatCheckFn_var_update(self):
        
        # rospy.loginfo('beatCheckFn_var_update is runing')
        #监听mqtt数据发送是否正常标识符，并进行处理
        if self.ele.flag_mqtt_comm_fail_pub==1:#1：发布mqtt数据通信异常状态
            
            if self.ele.connectionState==False:
                self.ob_mqtt_connect_fail_pub.publish(1)
                rospy.loginfo("flag_mqtt_comm_fail_pub:1")
            else:
                self.ob_mqtt_connect_fail_pub.publish(2) 
                rospy.loginfo("flag_mqtt_comm_fail_pub:2") 
            self.ele.flag_mqtt_comm_fail_pub=0
            self.mqtt_comm_fail_cnt = self.mqtt_comm_fail_cnt + 1 #记录mqtt_comm_fail的次数,用于降低连续请求网络连接的频率,减少流量消耗
            if self.mqtt_comm_fail_cnt>=50:
                self.ele.TH_cmd_repeat_timer=10 
            # rospy.loginfo("mqtt_comm_fail_cnt: "+str(self.mqtt_comm_fail_cnt))
       
        if self.ele.flag_mqtt_comm_fail_pub==2:#2：发布mqtt数据通信正常状态
            self.ob_mqtt_connect_fail_pub.publish(0) 
            self.ele.flag_mqtt_comm_fail_pub=0
            self.mqtt_comm_fail_cnt = 0 #记录mqtt_comm_fail的次数,用于降低连续请求网络连接的频率,减少流量消耗
            self.ele.TH_cmd_repeat_timer=0.5        
            rospy.loginfo("flag_mqtt_comm_fail_pub:0")

        #定时打印Bag时间戳
        self.cnt_bagrecord_Beat += self.checkBeat
        if self.cnt_bagrecord_Beat>=30:
            self.cnt_bagrecord_Beat-=30
            now_time = rospy.get_rostime()
            x = time.localtime(now_time.secs)#localtime参数为float类型，这里1317091800.0为float类型
            rospy.loginfo('now_time: '+time.strftime('%Y-%m-%d %H:%M:%S',x))
            # self.ob_record_bag_pub.publish(1)#启动一次bag包记录
            lt = time.localtime(time.time())
            self.ob_get_robot_time_pub.publish(lt.tm_hour)
        #repeat beatCheckFn
        self.check_var_update = threading.Timer(self.checkBeat, self.beatCheckFn_var_update)
        self.check_var_update.start()


    def mainloop(self):

        if self.nav_step == 1:#nav to the same floor pos
            
            #已经在相同的楼层，直接导航到目标点(房间或者充电点)

            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3)

                #设定机器人正常的行走TEB参数
                self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_normal, 'yaw_goal_tolerance': self.yaw_goal_tolerance_normal, 'max_vel_x': self.max_vel_x_normal, 'max_vel_theta':self.max_vel_theta_normal})

                if self.map_goal_id_record!=100001:#导航到充电点不需要判断是否楼层占用,因为机器人已经回到了原点
                    #不跨楼层，查询等待当前楼层占用清除及目标楼层避让点清除
                    if self.robot_for_education == False and self.robot_for_meal_delivery == False:
                        self.ele.getRobotState()
                        while(self.ele.isFloorOccupied(self.currentfloor) is True) or (self.ele.isAvoidOccupied(self.exp_floor) is True):
                            # rospy.loginfo("waiting for clear current floor & exp floor avoid occupied")
                            if self.ele.isFloorOccupied(self.currentfloor) is True:
                                self.ob_robot_delivery_status_pub.publish(2)
                            elif self.ele.isAvoidOccupied(self.exp_floor) is True:
                                self.ob_robot_delivery_status_pub.publish(3)
                            if self.ob_error_status!=0:
                                break
                            self.ele.getRobotState()

                    if self.ob_error_status==0:
                        # rospy.loginfo("current floor occupied & exp floor avoid occupied is clear")
                        self.ob_robot_delivery_status_pub.publish(0)                    
                    
                        #置位当前楼层被占用，清除当前楼层避让点被占用,清除电梯被占用
                        rospy.loginfo("set current floor occupied,clear avoid point occupied,clear elevator occupied")
                        self.floor_occ_status = self.currentfloor
                        self.avoidpoint_occ_status = 0
                        self.ele_occ_status = 0
                        self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
                
                self.search_pose_base = self.map_goal_pose_record#目标点坐标
                self.search_pose = self.map_goal_pose_record#目标点坐标

            goal = MoveBaseGoal() 
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.search_pose

            if self.ob_error_status == 0:
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2

            if tempresult == 1:#同楼层导航到房间或者原点或者充电点成功
                self.ob_nav_finish_pub.publish(1)
                # rospy.loginfo("navigation goal success")
                self.map_goal_moving = 0
                self.ob_in_moving_pub.publish(self.map_goal_moving)

                if self.map_goal_id_record==100002:#返航回到原点
                    if self.Delivery_Start_flag != 0:
                        self.Delivery_Start_flag = 0#记录派送启动标志位,主要用来判断返回原点后是否要记录派送结束时间和上传派送总时间
                        self.Delivery_TimeEnd = rospy.get_rostime()
                        self.Delivery_TimeTotal = self.Delivery_TimeEnd.secs - self.Delivery_TimeBegin.secs
                        self.ele.endDeliverReport(self.Delivery_TimeTotal,self.Delivery_TimeEnd.secs)
                        rospy.loginfo("Delivery_TimeTotal: "+str(self.Delivery_TimeTotal))
                        #++++++++++++++++++++++++派送云端统计 end++++++++++++++++++++++++

                    #返航回到原点,清除当前所有占用,机器人进入空闲状态
                    rospy.loginfo("end of delivery,clear all of occupied") 
                    self.ele_occ_status = 0
                    self.floor_occ_status = 0
                    self.avoidpoint_occ_status = 0
                    self.robot_working_status = 0
                    self.ob_robot_working_status_pub.publish(self.robot_working_status)
                    self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)

                    if self.ob_web_tool_runing_Flag==False:
                        if self.robot_for_meal_delivery == False:
                            #返回到原点后启动一次自动充电,机器人进入充电状态
                            self.ob_auto_recharge_pub.publish(True)#发送启动自动充电功能
                        
                elif self.map_goal_id_record==100001:#返航回到充电点
                    if self.in_recharing==0:
                        self.ob_auto_recharge_pub.publish(True)#发送启动自动充电功能                   

                if self.map_goal_id_record<100000:
                    self.nav_step = 15        #15：reach the room pos
                    self.nav_try_cnt = 0
                    self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                    rospy.loginfo("nav_step: "+str(self.nav_step))
                    #发送机器人状态给到mqtt程序
                    self.ele.getRobotStateFromRobot(self.ob_error_status,self.nav_step)
                    
                else:
                    self.nav_step = 0        #0: nothing 


            elif tempresult != 2:#同楼层导航到房间或者原点或者充电点失败
                
                self.nav_try_cnt += 1 #失败计数加1
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.nav_try_cnt:"+str(self.nav_try_cnt))

                if self.nav_try_cnt >= self.TH_nav_try_cnt:
                    self.nav_try_cnt=self.TH_nav_try_cnt
                    self.fun_find_near_pose_nav_path()#寻找目标导航点附近点的可行路线
                else:
                    #播放避让机器人提示音
                    self.fun_audio_play_pathfail()

        elif self.nav_step == 2:#2: nav to elevator wait pose
            # 导航到电梯前的位置点
            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3) 
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100001 + self.currentfloor * 1000
                resp = self.posemanage(poseoperation)
                if "FAIL" in resp.status:
                    rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.search_pose_base = resp.posedefine.pose#目标点坐标
                self.search_pose = self.search_pose_base#目标点坐标

                self.ele.getRobotState()
                while(self.ele.isFloorOccupied(self.currentfloor) is True) or (self.ele.isAvoidOccupied(self.exp_floor) is True) or (self.ele.isEleOccupied() is True):
                    # rospy.loginfo("waiting for clear current floor & exp floor avoid & elevator occupied")
                    if self.ele.isEleOccupied() is True:
                        self.ob_robot_delivery_status_pub.publish(1)
                    elif self.ele.isFloorOccupied(self.currentfloor) is True:
                        self.ob_robot_delivery_status_pub.publish(2)
                    elif self.ele.isAvoidOccupied(self.exp_floor) is True:
                        self.ob_robot_delivery_status_pub.publish(3)
                    if self.ob_error_status!=0:
                        break
                    self.ele.getRobotState()

                if self.ob_error_status==0:
                    # rospy.loginfo("current floor occupied & exp floor avoid & elevator occupied is clear")
                    self.ob_robot_delivery_status_pub.publish(0)

                    rospy.loginfo(" set current floor occupied & elevator occupied ")
                    self.ele_occ_status = 1
                    self.floor_occ_status = self.currentfloor
                    self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)  

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.search_pose
            if self.ob_error_status == 0:
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2
            if tempresult == 1:# 导航到电梯前等待电梯位置点
                self.nav_step = 3        #3: call elevator
                self.nav_try_cnt = 0
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))
            elif tempresult != 2:# 导航到电梯前的位置点
                self.nav_try_cnt += 1 #失败计数加1
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.:"+str(self.nav_try_cnt))
                if(self.nav_try_cnt >= self.TH_nav_try_cnt):
                    self.nav_try_cnt=self.TH_nav_try_cnt
                    self.fun_find_near_pose_nav_path()#寻找目标导航点附近点的可行路线
                else:
                    #播放避让机器人提示音
                    self.fun_audio_play_pathfail()

        elif self.nav_step == 3:#3: call elevator   
            
            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3) 

            #清当前楼层被占用标志
            self.floor_occ_status = 0
            self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)

            # TCP预约电梯++++++++++++++++++++++++++++++++++
            self.ele.elevatorCtrl(startingFloor=str(self.currentfloor), gotoFloor=str(self.exp_floor), upDown=str(self.elevator_up_down))
            self.ele.floor = "0"
            self.ele.frontdoor = "0"
            self.ele.reach_floor = False
            self.elevator_re_outcall_cnt = 0
            rospy.loginfo("---readElevatorState---")
            while((self.elevator_ack_flag is False) and (not rospy.is_shutdown()) and (self.ob_error_status == 0)):
                # rospy.loginfo("Waiting for elevator arriving floor,current floor:"+str(self.currentfloor))
                # 等待电梯到达
                # tcp查询电梯的状态
                self.ele.readElevatorState()
                # 上传机器人搭乘电梯状态信息
                self.fun_robot_take_elevotor_info_update(1)
                #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来

                #tcp等待电梯到达目标楼层并且门常开-----------------------------------------------------
                if (self.ele.floor == str(self.currentfloor)) and (self.ele.frontdoor == str(4)):
                    #电梯到达机器人所在楼层并且开门到位
                    # rospy.loginfo("arriving and door is open")
                    self.elevator_ack_flag = True
                    self.ele.reach_floor = False
                    self.elevator_re_outcall_cnt = 0
                else:
                    self.elevator_re_outcall_cnt = self.elevator_re_outcall_cnt + 1
                    if self.elevator_re_outcall_cnt > 60:
                        self.elevator_re_outcall_cnt = 0
                        rospy.loginfo("It`s over one minute,make a new appointment:outcall")
                        # TCP预约电梯++++++++++++++++++++++++++++++++++
                        self.ele.elevatorCtrl(startingFloor=str(self.currentfloor), gotoFloor=str(self.exp_floor), upDown=str(self.elevator_up_down))
                        rospy.sleep(0.5)
                        self.ele.floor = "0"
                        self.ele.frontdoor = "0"
                        self.ele.reach_floor = False

            # TCP预约电梯-------------------------------------
            if self.elevator_ack_flag is True:
                self.elevator_ack_flag = False
                self.nav_step = 4        #4: nav to elevator inside pose 
                self.nav_try_cnt = 0
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))

        elif self.nav_step == 4:#4: nav to elevator inside pose
            # 导航到电梯里面的点
            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3)

                #test code
                # self.ob_record_bag_pub.publish(5)#启动一次bag包记录

                # 上传机器人搭乘电梯状态信息
                #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
                self.fun_robot_take_elevotor_info_update(2)

                # 设置目标位置的容忍度，并且降低最大线速度与最大角速度
                self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_elevator , 'yaw_goal_tolerance': self.yaw_goal_tolerance_elevator, 'max_vel_x': self.max_vel_x_elevator , 'max_vel_theta': self.max_vel_theta_elevator})
                # rospy.sleep(0.2)
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100002 + self.currentfloor * 1000
                resp = self.posemanage(poseoperation)          
                if "FAIL" in resp.status:
                    rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.search_pose_base = resp.posedefine.pose#目标点坐标
                self.search_pose = self.search_pose_base#目标点坐标

            # tcp常按开门键
            self.ele.elevatorCtrl(fOpenCtrl='25')
            rospy.loginfo("---nav to elevator inside pose,hold the opendoor key---")

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.search_pose
            if self.ob_error_status == 0:
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2
            if tempresult == 1:# 导航到电梯里面的点
                rospy.loginfo("---reach the center of elevator---")
                self.nav_step = 5        #5: robot turn around
                self.nav_try_cnt = 0
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))

            elif tempresult != 2:# 导航到电梯里面的点失败       

                self.nav_try_cnt += 1
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.nav_try_cnt:"+str(self.nav_try_cnt))
                if self.nav_try_cnt >= 2:# 如果超过最大的尝试次数

                    #进入电梯失败，先导航回到当前楼层进电梯等待点,然后导航到当前楼层避让点                    
                    self.nav_step = 9        #9: get in elve fail,nav back to the elev wait point
                    self.nav_try_cnt = 0
                    self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                    rospy.loginfo("nav_step: "+str(self.nav_step))

        elif self.nav_step == 9:    #9: get in elve fail,nav back to the elev wait point

            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3) 

                #在这里不需要判断当前楼层是否被占用或者避让点被占用,因为以上情况是不会发生的!!!!
                #另外导航到楼层避让点操作不可以清空电梯占用,必须一直占用电梯,因为当前机器人的任务是乘梯,释放电梯会造成冲突!!!!
                # 设置目标位置的容忍度，并且降低最大线速度与最大角速度
                # self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_elevator, 'yaw_goal_tolerance': self.yaw_goal_tolerance_elevator, 'max_vel_x': self.max_vel_x_elevator, 'max_vel_theta': self.max_vel_theta_elevator})
                self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_normal, 'yaw_goal_tolerance': self.yaw_goal_tolerance_normal, 'max_vel_x': self.max_vel_x_normal, 'max_vel_theta':self.max_vel_theta_normal})
                   
                #上传乘梯信息
                #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
                self.fun_robot_take_elevotor_info_update(5)
                # rospy.sleep(4)#等待语音播放完成

                #先置位当前楼层避让点占用
                self.avoidpoint_occ_status = self.currentfloor
                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)

                # 导航去电梯前的位置点
                # rospy.sleep(0.2)
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100001 + self.currentfloor * 1000
                resp = self.posemanage(poseoperation)
                if "FAIL" in resp.status:
                    rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.search_pose_base = resp.posedefine.pose#目标点坐标
                self.search_pose = self.search_pose_base#目标点坐标

            # tcp常按开门键
            self.ele.elevatorCtrl(fOpenCtrl='25')
            rospy.loginfo("---get in fail and return to wait pos fail,hold the opendoor key---")

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.search_pose
            if self.ob_error_status == 0:
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2
            if tempresult == 1:# 导航到电梯前的位置点
                    
                #进入电梯失败，先导航回到当前楼层进电梯等待点,然后导航到当前楼层避让点                    
                self.nav_step = 10        #10: get in elve fail,nav to the floor avoid point
                self.nav_try_cnt = 0
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))

            elif tempresult != 2:# 导航到电梯前的位置点失败

                #返回电梯口等待点导航失败                        
                self.nav_try_cnt += 1#失败计数加1
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.nav_try_cnt:"+str(self.nav_try_cnt))
                if(self.nav_try_cnt >= self.TH_nav_try_cnt):
                    self.nav_try_cnt=self.TH_nav_try_cnt
                    self.fun_find_near_pose_nav_path()#寻找目标导航点附近点的可行路线
                else:
                    #播放避让机器人提示音
                    self.fun_audio_play_pathfail()
        
        elif self.nav_step == 10:    #10: get in elve fail,nav to the floor avoid point

            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3) 
                
                # 设置目标位置的容忍度，并且降低最大线速度与最大角速度
                # self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_elevator, 'yaw_goal_tolerance': self.yaw_goal_tolerance_elevator, 'max_vel_x': self.max_vel_x_elevator, 'max_vel_theta': self.max_vel_theta_elevator})
                self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_normal, 'yaw_goal_tolerance': self.yaw_goal_tolerance_normal, 'max_vel_x': self.max_vel_x_normal, 'max_vel_theta':self.max_vel_theta_normal})

                #松开开门按钮,释放电梯关门
                self.ele.elevatorCtrl(fOpenCtrl='0')
                rospy.loginfo("---get in fail and return to wait pos success,release openctrl---")

                #导航去当前楼层避让点
                # rospy.sleep(0.2)
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100005 + self.currentfloor * 1000
                resp = self.posemanage(poseoperation)
                if "FAIL" in resp.status:
                    rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.search_pose_base = resp.posedefine.pose#目标点坐标
                self.search_pose = self.search_pose_base#目标点坐标
                        
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()                
            goal.target_pose.pose = self.search_pose
            if self.ob_error_status == 0:
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2
            if tempresult == 1:#导航到当前楼层避让点

                rospy.sleep(10)#等待10秒让里面人或者货物出来

                #清除当前楼层避让点占用
                self.avoidpoint_occ_status = 0
                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
                self.nav_step = 2         #2: nav to elevator wait pose 
                self.nav_try_cnt = 0
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))

            elif tempresult != 2:#导航到当前楼层避让点失败

                #导航到楼层避让点失败
                self.nav_try_cnt += 1 #失败计数加1         
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.nav_try_cnt:"+str(self.nav_try_cnt))
                if(self.nav_try_cnt >= self.TH_nav_try_cnt):
                    self.nav_try_cnt=self.TH_nav_try_cnt
                    self.fun_find_near_pose_nav_path()#寻找目标导航点附近点的可行路线
                else:
                    #播放避让机器人提示音
                    self.fun_audio_play_pathfail()             

        elif self.nav_step == 5:#5: robot turn around

            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3) 
               
                # tcp松开开门按钮
                self.ele.elevatorCtrl(fOpenCtrl='0')
                self.ele.elevatorCtrl(fCloseCtrl='1')
                rospy.loginfo("---inside success,release the opendoor key and excu closedoor---")


            #上传乘梯信息
            #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
            self.fun_robot_take_elevotor_info_update(3)

            # 在电梯里面旋转，面向电梯门
            rospy.loginfo("turning around in the elevator")

            poseoperation=PoseOperation()
            poseoperation.opt   = "select"
            poseoperation.id    = 100004 + self.currentfloor * 1000
            resp = self.posemanage(poseoperation)
            if "FAIL" in resp.status:
                rospy.loginfo("cannot find the pose:"+str(poseoperation.id))

            _rot = [0] * 4
            _rot[0] = resp.posedefine.pose.orientation.x
            _rot[1] = resp.posedefine.pose.orientation.y
            _rot[2] = resp.posedefine.pose.orientation.z
            _rot[3] = resp.posedefine.pose.orientation.w  # -
            _euler1 = tf.transformations.euler_from_quaternion(_rot)
            _yaw_stop = _euler1[2]

            if _yaw_stop > math.pi:
                _yaw_stop -= 2 * math.pi
            if _yaw_stop < -math.pi:
                _yaw_stop += 2 * math.pi

            (_trans, _rot) = self.tflistener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            _euler1 = tf.transformations.euler_from_quaternion(_rot)
            _yaw_start = _euler1[2]

            _yaw_cur = _yaw_start
            _yaw_cur_last = _yaw_start

            tmp_vel = Twist()

            if _yaw_stop > _yaw_start:
                if(_yaw_stop - _yaw_start > math.pi):
                    _yaw_stop -= 2 * math.pi
                    tmp_vel.angular.z = -0.7
                else:
                    tmp_vel.angular.z = 0.7
            else:
                if(_yaw_stop - _yaw_start < -math.pi):
                    tmp_vel.angular.z = 0.7
                    _yaw_stop += 2 * math.pi
                else:
                    tmp_vel.angular.z = -0.7

            tb = rospy.get_time()
            delta_time = 0
            temp_time = 0
            # rospy.loginfo("robot turn around start!!!!")
            while((temp_time) < 30) and (not rospy.is_shutdown() and (self.ob_error_status == 0)):
                delta_time = rospy.get_time() - tb
                tb = rospy.get_time()
                temp_time += delta_time

                (_trans, _rot) = self.tflistener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
                _euler1 = tf.transformations.euler_from_quaternion(_rot)
                _yaw_cur = _euler1[2]

                if _yaw_cur - _yaw_cur_last > math.pi:
                    _yaw_stop += 2 * math.pi
                elif _yaw_cur - _yaw_cur_last < -math.pi:
                    _yaw_stop -= 2 * math.pi

                _yaw_cur_last = _yaw_cur
                # rospy.loginfo("yaw_current:"+str(_yaw_cur)+",yaw_stop:"+str(_yaw_stop ))
                
                if abs(_yaw_cur - _yaw_stop)<0.2:
                    if tmp_vel.angular.z <0:
                        tmp_vel.angular.z = -0.1
                    elif tmp_vel.angular.z >0:
                        tmp_vel.angular.z = 0.1

                if(tmp_vel.angular.z > 0) and (_yaw_cur+0.03 > _yaw_stop):
                    break
                elif(tmp_vel.angular.z < 0) and (_yaw_cur-0.03 < _yaw_stop):
                    break
                
                self.velpub.publish(tmp_vel)
                rospy.sleep(0.01)

            if self.ob_error_status == 0:
                # rospy.loginfo("robot turn around stop!!!!")
                tmp_vel.linear.x = 0
                tmp_vel.angular.z = 0
                self.velpub.publish(tmp_vel) 

                self.nav_step = 6         #6: exchange floor 
                self.nav_try_cnt = 0
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))
        elif self.nav_step == 6:      #6: exchange floor 
            
            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(10)  #test code          
           
            # 更改当前地图并且设置初始点
            self.ob_changing_current_map_pub.publish(True)
            rospy.loginfo("-------------change floor map-----------------------")
            self.reset_map = rospy.ServiceProxy('reset_map', reset_map)
            floor_test_src = "/home/ob/.ob/map/OB_floor_" + str(self.exp_floor) + ".yaml"  #目标楼层
            tempresult = self.reset_map(floor_test_src, "map")
            floor_test_src = "/home/ob/.ob/map/OB_floor_location_" + str(self.exp_floor) + ".yaml"  #目标楼层
            tempresult = self.reset_map(floor_test_src, "map_amcl")
            if tempresult.result is True:
                self.currentfloor = int(self.exp_floor)
                self.ob_current_floor_pub.publish(self.currentfloor)
                rospy.loginfo("self.currentfloor:"+str(self.currentfloor))

            rospy.sleep(3)

            #先在电梯内做个初始的重定位,等会电梯到了目标楼层后出来前再做一次YS重定位
            poseoperation=PoseOperation()
            poseoperation.opt   = "select"
            poseoperation.id    = 100004 + self.exp_floor * 1000
            resp = self.posemanage(poseoperation)
            if "FAIL" in resp.status:
                rospy.loginfo("cannot find the pose:"+str(poseoperation.id))

            self.ob_changing_current_map_pub.publish(True)
            rospy.sleep(1)
            rospy.loginfo('start localization!!')
            origin_pose = PoseWithCovarianceStamped()
            origin_pose.header.frame_id = "/map"
            origin_pose.pose.pose.position.x = resp.posedefine.pose.position.x
            origin_pose.pose.pose.position.y = resp.posedefine.pose.position.y
            origin_pose.pose.pose.position.z = resp.posedefine.pose.position.z
            origin_pose.pose.pose.orientation.x = resp.posedefine.pose.orientation.x    
            origin_pose.pose.pose.orientation.y = resp.posedefine.pose.orientation.y    
            origin_pose.pose.pose.orientation.z = resp.posedefine.pose.orientation.z    
            origin_pose.pose.pose.orientation.w = resp.posedefine.pose.orientation.w    
            origin_pose.pose.covariance[0] = 0.003
            origin_pose.pose.covariance[7] = 0.003
            # origin_pose.pose.covariance[14] = 1.0
            # origin_pose.pose.covariance[21] = 1.0
            # origin_pose.pose.covariance[28] = 1.0
            origin_pose.pose.covariance[35] = 0.001
            self.initialpose_pub.publish(origin_pose)
            rospy.sleep(1)
            self.ob_changing_current_map_pub.publish(False)

            # rospy.loginfo("getout elevator initialpose ")

            if self.ob_error_status == 0:
                self.nav_step = 7        #7: wait for elevatot reach exp floor
                self.nav_try_cnt = 0
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))
        elif self.nav_step == 7:        #7: wait for elevatot reach exp floor
            
            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3) 
                # self.ob_record_bag_pub.publish(0)#停止bag包记录
                # TCP预约电梯++++++++++++++++++++++++++++++++++
                self.ele.elevatorCtrl(startingFloor=str(self.currentfloor), gotoFloor=str(self.exp_floor), upDown=str(self.elevator_up_down))

            #tcp等待电梯到达目标楼层并且门常开++++++++++++++++++++++++++++++++++++++++++++++++++
            self.ele.floor = "0"
            self.ele.frontdoor = "0"
            self.ele.reach_floor = False
            self.elevator_re_outcall_cnt = 0
            rospy.loginfo("---readElevatorState---")
            while((self.elevator_ack_flag is False) and (not rospy.is_shutdown()) and (self.ob_error_status == 0)):
                # rospy.loginfo("Waiting for elevator arriving floor,exp_floor:"+str(self.exp_floor))
                # 等待电梯到达目标楼层
                # tcp查询电梯的状态
                self.ele.readElevatorState()
                #上传乘梯信息
                #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
                self.fun_robot_take_elevotor_info_update(3)
                
                #tcp等待电梯到达目标楼层并且门常开-----------------------------------------------------
                if (self.ele.floor == str(self.exp_floor)) and (self.ele.frontdoor == str(4)):
                    #电梯到达目标楼层并且开门到位
                    self.elevator_ack_flag = True
                    self.ele.reach_floor = False
                    # rospy.loginfo("arriving and door is open")
                    self.elevator_re_outcall_cnt = 0
                else:
                    self.elevator_re_outcall_cnt = self.elevator_re_outcall_cnt + 1
                    if self.elevator_re_outcall_cnt > 60:
                        self.elevator_re_outcall_cnt = 0
                        rospy.loginfo("It`s over half minute,make a new appointment:outcall")
                        # TCP预约电梯++++++++++++++++++++++++++++++++++
                        self.ele.elevatorCtrl(startingFloor=str(self.currentfloor), gotoFloor=str(self.exp_floor), upDown=str(self.elevator_up_down))
                        rospy.sleep(0.5)
                        self.ele.floor = "0"
                        self.ele.frontdoor = "0"
                        self.ele.reach_floor = False                 

            if self.elevator_ack_flag is True:
                self.elevator_ack_flag = False
                self.nav_step = 8         #8: nav to elelvator out pos
                self.nav_try_cnt = 0 
                self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                rospy.loginfo("nav_step: "+str(self.nav_step))

        elif self.nav_step == 8:        #8: nav to elelvator out pos 

            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3)
                
                #test code
                # self.ob_record_bag_pub.publish(5)#启动一次bag包记录

                #上传乘梯信息
                #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
                self.fun_robot_take_elevotor_info_update(4)

                # 设置目标位置的容忍度，并且降低最大线速度与最大角速度
                self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_elevator, 'yaw_goal_tolerance': self.yaw_goal_tolerance_elevator, 'max_vel_x': self.max_vel_x_elevator, 'max_vel_theta': self.max_vel_theta_elevator})
                rospy.sleep(0.2)

                #获取出电梯口点位姿
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100003 + self.currentfloor * 1000
                resp = self.posemanage(poseoperation)
                if "FAIL" in resp.status:
                    rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.search_pose_base = resp.posedefine.pose#目标点坐标
                self.search_pose = self.search_pose_base#目标点坐标


                # #到达目标楼层,然后做YS重定位,当评分满足要求或者次数超出时退出

                # #获取重定位点位姿
                # poseoperation=PoseOperation()
                # poseoperation.opt   = "select"
                # poseoperation.id    = 100004 + self.exp_floor * 1000
                # resp = self.posemanage(poseoperation)
                # if "FAIL" in resp.status:
                #     rospy.loginfo("cannot find the pose:"+str(poseoperation.id))

                # #启动一次YS重定位操作
                # #先求出方位角
                # _rot = [resp.posedefine.pose.orientation.x, resp.posedefine.pose.orientation.y, resp.posedefine.pose.orientation.z, resp.posedefine.pose.orientation.w]
                # self.yaw_cur = tf.transformations.euler_from_quaternion(_rot)[2]
                # self.yaw_cur = self.yaw_cur/math.pi*180 #弧度转角度
                
                # cnt_relocation=0
                # score_relocation = 0
                # while((cnt_relocation<10) and (score_relocation <0.75) and (self.ob_error_status == 0)):

                #     self.ob_localization = rospy.ServiceProxy('initial_pose_one', localization_srv)
                #     #rospy.loginfo("inside elevator wait for ob_localization")
                #     self.ob_localization.wait_for_service()
                #     #rospy.loginfo('found "ob_localization" service')
                #     self.ob_changing_current_map_pub.publish(True)
                #     # rospy.sleep(0.3)
                #     #rospy.loginfo('start localization!!')
                #     tempresult = self.ob_localization(resp.posedefine.pose.position.x,resp.posedefine.pose.position.y,self.yaw_cur)
                #     if tempresult.result is True:
                #         #rospy.loginfo("ob_localization finish,score："+ str(tempresult.score))
                #         score_relocation = tempresult.score
                #     self.ob_changing_current_map_pub.publish(False)

                #     cnt_relocation = cnt_relocation + 1
                #     #rospy.loginfo("cnt_relocation: "+str(cnt_relocation))

            if self.ob_error_status == 0:        
                # tcp常按开门键
                self.ele.elevatorCtrl(fOpenCtrl='25')
                rospy.loginfo("---nav to elelvator out pos,hold the opendoor key---")

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.search_pose
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2
            if tempresult == 1:#导航到出电梯口点
                
                 
                # self.ob_record_bag_pub.publish(0)#停止bag包记录

                # 成功导航出电梯
                  
                #上传乘梯信息
                #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
                self.fun_robot_take_elevotor_info_update(0)

                # tcp释放电梯控制权
                self.ele.elevatorCtrl(fOpenCtrl='0')
                rospy.loginfo("---get out success,release the opendoor key---")

                #查询当前楼层是否被其他机器人占用
                self.ele.getRobotState()

                if (self.ele.isFloorOccupied(self.currentfloor) is True) :
                    #如果当前楼层被占用，去往楼层避让点 
                    self.nav_step = 11        #11: current floor is occupied,nav to the avoid pos 
                    self.nav_try_cnt = 0
                    self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                    rospy.loginfo("nav_step: "+str(self.nav_step))
                else:
                    #如果当前楼层未被占用，继续执行同层导航任务
                    self.nav_step = 1        #1: nav to the same floor pos 
                    self.nav_try_cnt = 0
                    self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                    rospy.loginfo("nav_step: "+str(self.nav_step))

            elif tempresult != 2:#导航到出电梯口点失败

                #*******************************如果第一次or多次导航失败，是否考虑为切换地图，地图偏移造成，增加左右原地旋转将粒子收敛用以修正定位偏移,同样要考虑关闭AMCL定位跳变检测
                self.nav_try_cnt += 1
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.nav_try_cnt:"+str(self.nav_try_cnt))
                if self.nav_try_cnt >= 2:
                    # rospy.loginfo("---get out fail ,ge back to the centre---")

                    self.nav_step = 12        #12: get out elve fail,nav back to the elev relocation point
                    self.nav_try_cnt = 0
                    self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                    rospy.loginfo("nav_step: "+str(self.nav_step))

        elif self.nav_step == 12:        #12: get out elve fail,nav back to the elev relocation point
            
            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3)
                
                # 设置目标位置的容忍度，并且降低最大线速度与最大角速度
                # self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_elevator, 'yaw_goal_tolerance': self.yaw_goal_tolerance_elevator, 'max_vel_x': self.max_vel_x_elevator, 'max_vel_theta': self.max_vel_theta_elevator})
                self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_normal, 'yaw_goal_tolerance': self.yaw_goal_tolerance_normal, 'max_vel_x': self.max_vel_x_normal, 'max_vel_theta':self.max_vel_theta_normal})

                #上传乘梯信息
                #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
                self.fun_robot_take_elevotor_info_update(6)

                #出电梯失败，导航回到电梯内重定位位置点
                # rospy.sleep(0.2)
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100004 + self.currentfloor * 1000
                resp = self.posemanage(poseoperation)
                if "FAIL" in resp.status:
                    rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.search_pose_base = resp.posedefine.pose#目标点坐标
                self.search_pose = self.search_pose_base#目标点坐标

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.search_pose
            if self.ob_error_status == 0:
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2
            if tempresult == 1:#导航回到电梯内重定位位置点
                rospy.loginfo("---get out fail ,ge back to the centre success---")

                # tcp释放电梯控制权
                self.ele.elevatorCtrl(fOpenCtrl='0')
                rospy.loginfo("---get out fail and return to inside,release openctrl---")
                
                rospy.loginfo("---waitting 10s then recall the elevator---")
                rospy.sleep(10)#等待10秒后再重新呼梯,必须要等待不能连续呼梯,否则电梯无法关门影响正常使用!!!
        
                # tcp召唤电梯+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                self.ele.elevatorCtrl(startingFloor=str(self.currentfloor), gotoFloor=str(self.exp_floor), upDown=str(self.elevator_up_down))
                self.ele.floor = "0"
                self.ele.frontdoor = "0"
                self.ele.reach_floor = False
                self.elevator_re_outcall_cnt = 0
                # rospy.loginfo("---outcall---"+str(self.exp_floor))
                rospy.loginfo("---readElevatorState---")
                while((self.elevator_ack_flag is False) and (not rospy.is_shutdown()) and (self.ob_error_status == 0)):
                    # rospy.loginfo("Waiting for elevator arriving floor")
                    # 等待电梯到达
                    # tcp查询电梯的状态
                    self.ele.readElevatorState()
                    #上传乘梯信息
                    #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
                    self.fun_robot_take_elevotor_info_update(3)
                            
                    #tcp等待电梯到达目标楼层并且门常开-----------------------------------------------------
                    if (self.ele.floor == str(self.currentfloor)) and (self.ele.frontdoor == str(4)):
                        #电梯到达机器人所在楼层并且开门到位
                        self.elevator_ack_flag = True
                        self.ele.reach_floor = False
                        self.elevator_re_outcall_cnt = 0
                    else:
                        self.elevator_re_outcall_cnt = self.elevator_re_outcall_cnt + 1
                        if self.elevator_re_outcall_cnt > 60:
                            self.elevator_re_outcall_cnt = 0
                            rospy.loginfo("It`s over one minute,make a new appointment:outcall")
                            # TCP预约电梯++++++++++++++++++++++++++++++++++
                            self.ele.elevatorCtrl(startingFloor=str(self.currentfloor), gotoFloor=str(self.exp_floor), upDown=str(self.elevator_up_down))
                            rospy.sleep(0.5)
                            self.ele.floor = "0"
                            self.ele.frontdoor = "0"
                            self.ele.reach_floor = False

                # tcp预约电梯-------------------------------------------------------------------           
                if self.elevator_ack_flag is True:
                    self.elevator_ack_flag = False
                    self.nav_step = 8        #8: nav to elelvator out pos 
                    self.nav_try_cnt = 0
                    self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                    rospy.loginfo("nav_step: "+str(self.nav_step))

            elif tempresult != 2:#导航回到电梯内重定位位置点失败

                rospy.loginfo("---try to go back to the centre of elevator---")  
                self.nav_try_cnt += 1 #失败计数加1
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.nav_try_cnt:"+str(self.nav_try_cnt))
                if(self.nav_try_cnt >= self.TH_nav_try_cnt):
                    self.nav_try_cnt=self.TH_nav_try_cnt
                    # self.fun_find_near_pose_nav_path()#寻找目标导航点附近点的可行路线
                    # rospy.loginfo("navigation enter error status(stop)")
                    self.ob_nav_finish_pub.publish(2)
                    if self.ob_web_tool_runing_Flag==False:#非WEB部署调试模式下发送定位失败
                        self.ob_location_status_pub.publish(False)
                        # rospy.sleep(1)
                else:
                    #播放避让机器人提示音
                    self.fun_audio_play_pathfail()

        elif self.nav_step == 11:        #11: current floor is occupied,nav to the avoid pos
        
            #从电梯出来后发现当前楼层被占用,导航到楼层避让点
            if self.flag_nav_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_nav_step_first_run = False
                # rospy.sleep(3)

                #设定机器人正常的行走TEB参数
                self.drclient.update_configuration({'xy_goal_tolerance': self.xy_goal_tolerance_normal, 'yaw_goal_tolerance': self.yaw_goal_tolerance_normal, 'max_vel_x': self.max_vel_x_normal, 'max_vel_theta': self.max_vel_theta_normal})

                #当前楼层避让点占用置位
                self.avoidpoint_occ_status = self.currentfloor
                self.ele_occ_status = 0 #清电梯被占用标志
                rospy.loginfo("clear elevator occupied")
                self.ele.pubRobotState(self.ele_occ_status,self.floor_occ_status,self.avoidpoint_occ_status,self.robot_working_status)
                rospy.loginfo("current floor is occupied ,set avoid point occupied status")

                # 导航去当前楼层避让点
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100005 + self.currentfloor * 1000
                resp = self.posemanage(poseoperation)
                if "FAIL" in resp.status:
                    rospy.loginfo("cannot find the pose:"+str(poseoperation.id))
                self.search_pose_base = resp.posedefine.pose#目标点坐标
                self.search_pose = self.search_pose_base#目标点坐标

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.search_pose
            if self.ob_error_status == 0:
                tempresult = self.goto(goal) # 将目标位置发送到goto函数，一直在这里等待导航完成后结果返回
            else:
                tempresult=2
            if tempresult == 1:# 导航到当前楼层避让点

                #查询等待当前楼层占用清除
                self.ele.getRobotState()
                while(self.ele.isFloorOccupied(self.currentfloor) is True) :
                    # rospy.loginfo("waiting for clear current floor  occupied")
                    self.ob_robot_delivery_status_pub.publish(2)
                    if self.ob_error_status!=0:
                        break 
                    self.ele.getRobotState()
                    
                if self.ob_error_status==0:
                    #在异常发生情况下不能变更nav_step变量,必须在异常处理函数中更新nav_step变量
                    # rospy.loginfo("current floor occupied is clear ,mission keep ")
                    self.ob_robot_delivery_status_pub.publish(0)
                    self.nav_step = 1#1: nav to the same floor pos 
                    self.nav_try_cnt = 0
                    self.flag_nav_step_first_run = True#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                    rospy.loginfo("nav_step: "+str(self.nav_step))

            elif tempresult != 2:# 导航到当前楼层避让点失败
                self.nav_try_cnt += 1 #失败计数加1
                rospy.loginfo("self.nav_step:"+str(self.nav_step)+",self.nav_try_cnt:"+str(self.nav_try_cnt))
                if(self.nav_try_cnt >= self.TH_nav_try_cnt):
                    self.nav_try_cnt=self.TH_nav_try_cnt
                    self.fun_find_near_pose_nav_path()#寻找目标导航点附近点的可行路线
                else:
                    #播放避让机器人提示音
                    self.fun_audio_play_pathfail()

    #寻找目标导航点附近点的可行路线
    def fun_find_near_pose_nav_path(self):

        rospy.loginfo("search near nav pos")
        self.find_near_pose_nav_path = False#是否找到附近可导航路径点标识符


        #清空costmap，重新开始尝试规划路线
        clearcostmaps_srv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        tempresult = clearcostmaps_srv()
        rospy.loginfo("clear costmap")
        rospy.sleep(2)

        
        # print("self.search_pose_base")        
        # print(self.search_pose_base)
        self.search_pose = copy.deepcopy(self.search_pose_base)    
        _rot = [self.search_pose_base.orientation.x, self.search_pose_base.orientation.y, self.search_pose_base.orientation.z, self.search_pose_base.orientation.w]
        self.search_pose_base_angle = tf.transformations.euler_from_quaternion(_rot)[2]

        self.tflistener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(60))
        (_trans, _rot) = self.tflistener.lookupTransform("/map", "/base_footprint", rospy.Time(0))

        tempstartx = PoseStamped()
        tempstartx.header.frame_id = "/map"
        tempstartx.header.stamp = rospy.Time.now()
        tempstartx.pose.position.x = _trans[0]
        tempstartx.pose.position.y = _trans[1]
        tempstartx.pose.position.z = _trans[2]
        tempstartx.pose.orientation.x = _rot[0]    # Makes the origin quaternion valid.
        tempstartx.pose.orientation.y = _rot[1]    # Makes the origin quaternion valid.
        tempstartx.pose.orientation.z = _rot[2]    # Makes the origin quaternion valid.
        tempstartx.pose.orientation.w = _rot[3]    # Makes the origin quaternion valid.

        tempgoalx = PoseStamped()
        tempgoalx.header.frame_id = "/map"
        tempgoalx.header.stamp = rospy.Time.now()

        tempzzzx = (self.search_pose_base.position.x) #原来的目标点
        tempzzzy = (self.search_pose_base.position.y) #原来的目标点
        tempangle = self.search_pose_base_angle #原来的目标点的角度
        tempradius = self.search_radius #每个邻点的距离

        for tempi in range(0, len(self.search_table)):
            self.search_pose.position.x = tempzzzx + tempradius * self.search_table[tempi][0] * math.cos(tempangle) - tempradius * self.search_table[tempi][1] * math.sin(tempangle) 
            self.search_pose.position.y = tempzzzy + tempradius * self.search_table[tempi][0] * math.sin(tempangle) + tempradius * self.search_table[tempi][1] * math.cos(tempangle)

            tempgoalx.pose = self.search_pose
            tempplan = self.getglobalplan(tempstartx, tempgoalx, 0.1) #获取全局路径，0.1为目标位置的容忍度

            self.planning_length = len(tempplan.plan.poses) # 获取全局路径的长度，通过长度来判断路径规划是否成功
            # break#test code
            if(self.planning_length != 0):
                # rospy.loginfo("planning success")
                self.find_near_pose_nav_path = True
                rospy.loginfo("search near pos nav path success")
                print(self.search_pose)
                break

        if self.find_near_pose_nav_path is False:
            #目标点附近没有找到可行的位置
            rospy.loginfo("search near pos nav path fail")
            if self.map_goal_id_record<100000 and (self.nav_step == 1 or self.nav_step == 2 or self.nav_step == 10 or self.nav_step == 11):
                #导航到房间过程，如果发现导航失败则尝试返回原点
                rospy.loginfo("can not reach exp room , go back to the orgin")
                tempInt32 = Int32()
                tempInt32.data = 100002
                self.ob_map_goal_id_pub.publish(tempInt32)
            else:
                # rospy.loginfo("navigation enter error status(stop)")
                self.ob_nav_finish_pub.publish(2)
                if self.ob_web_tool_runing_Flag==False:#非WEB部署调试模式下发送定位失败
                    self.ob_location_status_pub.publish(False)
                    # rospy.sleep(1)


    # 上传机器人搭乘电梯状态信息
    def fun_robot_take_elevotor_info_update(self,data_update):

        elevator_info = Twist()
        #linear.x ： 0:空闲  1：等待电梯 2：进电梯过程中 3：在电梯内 4：出电梯过程 5：等待进入电梯，但进不去  6：等待出电梯，但出不来
        elevator_info.linear.x = data_update  
        elevator_info.linear.y = int(self.ele.floor)
        elevator_info.linear.z = self.exp_floor
        self.ob_elevator_info_pub.publish(elevator_info)
        # rospy.loginfo("ob_elevator_info_pub:"+str(data_update))

    #播放避让机器人提示音
    def fun_audio_play_pathfail(self):

        if self.nav_try_cnt%2!=0:
            self.infovoicepub.publish("pathplan_fail")
            rospy.loginfo("audio play:pathplan_fail")

    # 让机器人旋转指定的角度，单位：度
    def ob_turn_around_cb(self, data):
        if(self.goal_sent is False) and (self.ob_turn_around_flag is False):
            self.ob_turn_around_flag = True
            tmp_vel = Twist()
            if(data.data > 0):
                tmp_vel.angular.z = 0.1
            else:
                tmp_vel.angular.z = -0.1
            (_trans, _rot) = self.tflistener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
            _euler1 = tf.transformations.euler_from_quaternion(_rot)
            _yaw_start = _euler1[2]
            _yaw_stop = _yaw_start + data.data * math.pi / 180.0
            _yaw_cur = _yaw_start
            _yaw_cur_last = _yaw_start
            # print _yaw_start, _yaw_stop, _yaw_cur, _yaw_cur_last

            tb = rospy.get_time()
            delta_time = 0
            temp_time = 0
            while((temp_time) < (data.data / tmp_vel.angular.z * 3)) and (not rospy.is_shutdown()) and (self.goal_sent is False) and (self.ob_error_status == 0):
                delta_time = rospy.get_time() - tb
                tb = rospy.get_time()
                temp_time += delta_time

                (_trans, _rot) = self.tflistener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
                _euler1 = tf.transformations.euler_from_quaternion(_rot)
                _yaw_cur = _euler1[2]

                if _yaw_cur - _yaw_cur_last > math.pi:
                    _yaw_stop += 2 * math.pi
                elif _yaw_cur - _yaw_cur_last < -math.pi:
                    _yaw_stop -= 2 * math.pi

                _yaw_cur_last = _yaw_cur
                
                # print "_yaw_stop", _yaw_s/top, "_yaw_cur", _yaw_cur
                
                if(tmp_vel.angular.z > 0) and (_yaw_cur > _yaw_stop):
                    break
                if(tmp_vel.angular.z < 0) and (_yaw_cur < _yaw_stop):
                    break
                
                self.velpub.publish(tmp_vel)
                rospy.sleep(0.01)

            tmp_vel.linear.x = 0
            tmp_vel.angular.z = 0
            self.velpub.publish(tmp_vel)
            
            self.ob_turn_around_flag = False

    def shutdown(self):
        if self.goal_sent is True:
            self.cancelgoalflag = True
            self.move_base.cancel_goal()
        rospy.loginfo("ob_nav.py is Stop")
        rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node('auto_map_nav', anonymous=True)
    navigator = GoToPose()
    
    if navigator.robot_for_education == False:
        #启动MQTT连接和心跳机制
        navigator.startHeartBeat()
    navigator.startHeartBeat_var_update()

    while not rospy.is_shutdown():    
        navigator.mainloop()
        rospy.sleep(0.1)    
    rospy.spin()

