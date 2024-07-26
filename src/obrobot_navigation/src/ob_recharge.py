#!/usr/bin/env python
#encoding=utf-8

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int32, UInt16, Float32, String, Bool ,Byte
import tf
from obrobot_navigation.msg import PoseDefine,PoseOperation
from obrobot_navigation.srv import *
import math
from math import sqrt, pow

import time
import getpass

import copy

import json
from collections import OrderedDict
from map_server.srv import reset_map
from zys_localization.srv import localization_srv

class Recharge():
    def __init__(self):
        rospy.loginfo("......ob_recharge.py is running.....")
        self.recharge_pos = Pose()
        self.recharge_pos.orientation.w = 1
        self.recharge_pos_angle = 0.0
        rospy.on_shutdown(self.shutdown)

        self.robot_for_education = True#True #科教版机器人功能标识
        self.robot_for_meal_delivery = False#False #送餐机器人功能标识
        self.robot_for_guest_greeting = False#False #迎宾机器人功能标识

        self.move_to_charge_pos_success_flag = True #是否成功导航
        self.move_to_charge_pos_finish_flag = True #是否完成导航，不一定成功
        self.ir_data = hex(0)
        self.angular_vel = 0.1#红外对准时的角速度
        self.linear_vel = -0.02 #红外对准时的后退速度
        
        #0：空闲状态或者在后退对齐充电桩状态，
        #2：正在匹配充电桩，
        #3：正在充电；
        #5：电池充满； 
        #6：2个充电器同时接入
        self.recharge_status = 0 #底层的充电状态，
        
        self.forward_vel = 0.06 #退出充电的前进速度
        self.forward_time = 5 #退出充电的前进时间
        self.recharge_status_top = 5 # 上层的充电状态，3：正在充电；5：电池充满； 6：2个充电器同时接入
        
        # 根据红外得到的线速度表
        self.vel_table = [
        [self.linear_vel, self.linear_vel,	self.linear_vel,	self.linear_vel,	0,	self.linear_vel,	0,	self.linear_vel],
        [0,	0,	0,	0,	0,	0,	0,	0],
        [self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel],
        [0,	0,	self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,],
        [0,	self.linear_vel,	self.linear_vel,	0,	0,	0,	0,	0],
        [self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	self.linear_vel],
        [self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel],
        [0,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel]]

        # 根据红外得到的角速度表
        self.angular_table = [
        [0,	0,	0,	0,	-self.angular_vel,	0,	-self.angular_vel,	0],
        [self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel],
        [0,	self.angular_vel,	0,	0,	0,	0,	0,	0],
        [self.angular_vel,	self.angular_vel,	0,	self.angular_vel,	0,	0,	0,	0],
        [-self.angular_vel,	0,	0,	-self.angular_vel,	-self.angular_vel,	-self.angular_vel,	-self.angular_vel,	-self.angular_vel],
        [0,	self.angular_vel,	0,	0,	self.angular_vel,	0,	0,	0],
        [self.angular_vel,	0,	0,	0,	0,	0,	-self.angular_vel,	0],
        [self.angular_vel,	self.angular_vel,	0,	0,	0,	0,	0,	0]]

        self.obstaclefront = False #前方是否有障碍物   
        self.obstaclefront_last = False #前方是否有障碍物 
        self.obstaclefront_clear_cnt = 0 #清除前方有障碍物标志计数器  

        self.relocationsucess_flag = False   #重定位是否成功
        self.relocationfinish_flag = False   #重定位完成标识符，只有完成才进入自动充电
        self.find_ir_init_flag = False #两边是否都找到中间的红外标识符

        self.recharge_try_cnt = 0 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
        self.lost_ir_cnt = 0
        self.wait_matching_cnt=0
        self.get_ir_and_turning_back_cnt=0
        self.wait_top_recharge_cnt=0

        self.flag_low_voltage = 0   #低电量标识符,0表示没有低电量,0x01表示底盘电池低电量,0x02表示上面屏幕电池低电量
        self.car_voltage_value = 0 # 底层电池电压
        self.car_voltage_value_start = 470# 底层电池的充电起始时刻电压

        self.car_voltage_value_top = 530 # 上层电池电压
        self.car_voltage_value_start_top = 530 # 上层电池的充电起始时刻电压

        #0: idle 
        #1: nav to the recharge pos 
        #2: find the ir data 
        #3: matching the charger
        #4: detect recharge success
        #5: into recharging status and detect exit charging
        #6: exit the charging status
        self.recharge_step = 0 
        self.flag_recharge_step_first_run = True
        
        self.cnt_lase_scan = 0
        #LTME02A雷达的参数
        # self.length_point=1535#;%激光雷达点数
        # self.sum_angle_point=math.pi*3/2#;%激光雷达扫描角度范围
        # self.step_angle_point=self.sum_angle_point/(self.length_point-1)#;%激光雷达角度分辨率
        # self.start_angle_point_laser=-math.pi*0.75#;%激光雷达第一个点对应的角度
        # self.value_A=0.173#;%三角形边A的长度，也是激光雷达偏离车中心原点的距离
        # self.TH_value_obstacle=0.10#;%设定的障碍物距离阈值
        # self.value_radius_car=0.25#;%设定车的半径

        #G4雷达参数
        self.length_point=909#;%激光雷达点数
        self.sum_angle_point=math.pi*2#;%激光雷达扫描角度范围
        self.step_angle_point=self.sum_angle_point/(self.length_point-1)#;%激光雷达角度分辨率
        self.start_angle_point_laser=-math.pi#;%激光雷达第一个点对应的角度
        self.value_A=0.12#;%三角形边A的长度，也是激光雷达偏离车中心原点的距离
        self.TH_value_obstacle=0.20#;%设定的障碍物距离阈值
        self.value_radius_car=0.25#;%设定车的半径

        #获取充电点位置信息标识符
        self.get_recharge_pos_info=False

        # 订阅的话题
        rospy.Subscriber('ob_nav_finish', Int16, self.ob_nav_finish_cb) 
        rospy.Subscriber('ob_auto_recharge', Bool, self.ob_auto_recharge_cb) 
        rospy.Subscriber('recharge_status', Int16, self.recharge_status_cb)
        rospy.Subscriber('recharge_status_top', Int16, self.recharge_status_top_cb)
        rospy.Subscriber('voltage_value', Int32, self.recharge_value_cb)
        rospy.Subscriber('voltage_top_value', Int32, self.recharge_value_top_cb)
        rospy.Subscriber('ob_rechargePose_current', Bool, self.ob_rechargePose_current_cb) 
        rospy.Subscriber('ir_data', Int16, self.ir_data_cb)
        rospy.Subscriber("scan", LaserScan, self.scan_cb)      
        rospy.Subscriber("ob_relocation_finish", Bool, self.ob_relocation_finish_cb)
        # rospy.Subscriber('ob_in_delivery', Int16, self.ob_in_delivery_cb)
        # self.in_delivery = 0 # 是否正在执行派送任务 1：正在执行派送任务   
        # rospy.Subscriber('ob_in_moving', Int16, self.ob_in_moving_cb)
        # self.in_moving = 0 # 是否正在移动 1：正在移动
        rospy.Subscriber('ob_robot_working_status', Int16, self.ob_robot_working_status_cb)       
        self.robot_working_status = 0
        rospy.Subscriber('ob_error_status_driver', Int16, self.ob_error_status_driver_cb)
        self.ob_error_status = 0
        #订阅是否开启迎宾模式话题
        # rospy.Subscriber("ob_welcome", Int16, self.ob_welcome_cb)
        # self.ob_welcome_flag = 0
        #订阅退出充电桩后导航目标点话题
        rospy.Subscriber('ob_map_goal_id_outcharge', Int32, self.ob_map_goal_id_outcharge_cb)
        self.ob_map_goal_id_outcharge = 0
        rospy.Subscriber('ob_web_tool_runing', Bool, self.ob_web_tool_runing_cb)
        self.ob_web_tool_runing_Flag=False
        

        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
        file_external_cfg = json.load(file)
        file.close()
        self.chargetime = file_external_cfg["recharge_time"]
        self.uiinterfacepassword = file_external_cfg["password"]
        self.exit_chargetime = file_external_cfg["exit_recharge_time"]   
        #订阅设置机器人操作码话题
        rospy.Subscriber("ob_set_uiinterface_password", String, self.ob_set_uiinterface_password_cb)
        #订阅查询机器人操作码话题
        rospy.Subscriber("ob_get_uiinterface_password", Bool, self.ob_get_uiinterface_password_cb)
        #发布机器人操作码话题
        self.ob_uiinterfacepassword_pub = rospy.Publisher("ob_uiinterfacepassword", String, queue_size = 5)
        
        #订阅设置机器人自动充电时间话题
        rospy.Subscriber("ob_set_rechargeTime", String, self.ob_set_rechargeTime_callback)
        #订阅查询机器人自动充电时间话题
        rospy.Subscriber("ob_get_rechargeTime", Bool, self.ob_get_rechargeTime_callback)
        #发布机器人自动充电时间话题
        self.ob_rechargeTime_pub = rospy.Publisher("ob_rechargeTime", String, queue_size = 5)
        
        #订阅设置机器人退出充电时间话题
        rospy.Subscriber("ob_set_exit_rechargeTime", String, self.ob_set_exit_rechargeTime_cb)
        #订阅查询机器人退出充电时间话题
        rospy.Subscriber("ob_get_exit_rechargeTime", Bool, self.ob_get_exit_rechargeTime_cb)
        #发布机器人退出充电时间话题
        self.ob_exit_rechargeTime_pub = rospy.Publisher("ob_exit_rechargeTime", String, queue_size = 5)

        # 发布的话题
        #0：空闲
        #1：匹配充电桩
        #2：正在充电
        #充电状态发布话题，只在状态切换时才发布
        self.recharge_handle_pub = rospy.Publisher("recharge_handle", Int16, queue_size = 5, latch=True)
        self.in_recharing = 0 # 充电过程状态  0:not recharge 1:going to recharge point 2:matching the ir 3:start recharging
      
        self.ob_map_goal_pub = rospy.Publisher("ob_map_goal", Pose, queue_size = 5)   
        #发送机器人定位状态话题：定位成功为true,定位失败为false
        self.ob_location_status_pub = rospy.Publisher("ob_location_status", Bool, queue_size = 5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5) 
        self.ob_map_goal_id_pub = rospy.Publisher("ob_map_goal_id", Int32, queue_size = 5)
        self.ob_rechargeTime_pub = rospy.Publisher("ob_rechargeTime", String, queue_size = 5)
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 5)
        self.ob_exit_recharge_pub = rospy.Publisher("ob_exit_recharge", Byte, queue_size = 5)
        self.ob_obstaclefront_pub = rospy.Publisher('ob_obstaclefront', Bool, queue_size = 5) 

        self.ob_changing_current_map_pub = rospy.Publisher('ob_changing_current_map', Bool, queue_size = 5) # 切换地图中
        
        self.infovoicepub = rospy.Publisher('ob_info_voice_play', String, queue_size = 5) 


        self.posemanage = rospy.ServiceProxy('posemanage_server', PoseManage)
        rospy.loginfo("ob_recharge.py wait for posemanage_server")
        rospy.wait_for_service('posemanage_server')

        self.tflasertoodom = tf.TransformListener()

        self.ob_recharge_control_status_pub = rospy.Publisher("ob_recharge_control_status", Bool, queue_size = 5) 
        
        self.ob_current_floor_pub = rospy.Publisher("ob_current_floor", Int16, queue_size = 5, latch=True)
        rospy.Subscriber("ob_current_floor", Int16, self.current_floor_cb)
        self.currentfloor = 1


        #获取开机点坐标
        self.recharge_poseoperation=PoseOperation()
        self.recharge_poseoperation.opt   = "select"
        self.recharge_poseoperation.id    = 100000

        self.recharge_resp = self.posemanage(self.recharge_poseoperation)
        self.recharge_position = Pose()
        self.recharge_position.position.x = self.recharge_resp.posedefine.pose.position.x
        self.recharge_position.position.y = self.recharge_resp.posedefine.pose.position.y

        self.tflistener = tf.TransformListener()
        self.tflistener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(60))

        rospy.sleep(3)


    # 获取当前的楼层
    def current_floor_cb(self, data):
        self.currentfloor = int(data.data)


    def ob_map_goal_id_outcharge_cb(self, data):
        self.ob_map_goal_id_outcharge = data.data
    
    # def ob_welcome_cb(self, data):
    #     self.ob_welcome_flag = data.data

    def ob_error_status_driver_cb(self, data):
        self.ob_error_status = data.data

    # def ob_in_delivery_cb(self, data):
    #     self.in_delivery = data.data

    # def ob_in_moving_cb(self, data):
    #     self.in_moving = data.data

    def ob_robot_working_status_cb(self, data):
        self.robot_working_status = data.data


    def ob_web_tool_runing_cb(self, data):
        self.ob_web_tool_runing_Flag = data.data


    # 设置UI界面操作验证码
    def ob_set_uiinterface_password_cb(self, data):
        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
        load_dict = json.load(file,object_pairs_hook=OrderedDict)
        file.close()
        load_dict["password"] = data.data
        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'w')
        json.dump(load_dict,file,indent=4)
        file.close()
        self.uiinterfacepassword = data.data

    # 获取UI界面操作验证码
    def ob_get_uiinterface_password_cb(self, data):
        if(data.data is True):
            file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
            load_dict = json.load(file,object_pairs_hook=OrderedDict)
            file.close()
            self.uiinterfacepassword = load_dict["password"]
            # print("self.uiinterfacepassword:", self.uiinterfacepassword)
            tmpstr = String()
            tmpstr.data = self.uiinterfacepassword
            self.ob_uiinterfacepassword_pub.publish(tmpstr)



    # 设置充电时间
    def ob_set_rechargeTime_callback(self, data):
        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
        load_dict = json.load(file,object_pairs_hook=OrderedDict)
        file.close()
        load_dict["recharge_time"] = data.data
        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'w')
        json.dump(load_dict,file,indent=4)
        file.close()
        self.chargetime = data.data
        print("set self.chargetime:", self.chargetime)
    # 获取充电时间
    def ob_get_rechargeTime_callback(self, data):
        if(data.data is True):
            file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
            load_dict = json.load(file,object_pairs_hook=OrderedDict)
            file.close()
            self.chargetime = load_dict["recharge_time"]
            print("get self.chargetime:", self.chargetime)
            tmpstr = String()
            tmpstr.data = self.chargetime
            self.ob_rechargeTime_pub.publish(tmpstr)

    # 设置退出充电时间
    def ob_set_exit_rechargeTime_cb(self, data):
        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
        load_dict = json.load(file,object_pairs_hook=OrderedDict)
        file.close()
        load_dict["exit_recharge_time"] = data.data
        file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'w')
        json.dump(load_dict,file,indent=4)
        file.close()
        self.exit_chargetime = data.data
        print("set self.exit_chargetime:", self.exit_chargetime)
    # 获取退出充电时间
    def ob_get_exit_rechargeTime_cb(self, data):
        if(data.data is True):
            file = open("/home/" + getpass.getuser() + "/.ob/setting/external_cfg.json",'r')
            load_dict = json.load(file,object_pairs_hook=OrderedDict)
            file.close()
            self.exit_chargetime = load_dict["exit_recharge_time"]
            print("get self.exit_chargetime:", self.exit_chargetime)
            tmpstr = String()
            tmpstr.data = self.exit_chargetime
            self.ob_exit_rechargeTime_pub.publish(tmpstr)





    def ob_relocation_finish_cb(self, data):
        if(data.data is True):
            self.relocationsucess_flag = True
        self.relocationfinish_flag = True

    def scan_cb(self, data):
        self.cnt_lase_scan = self.cnt_lase_scan + 1
        if self.cnt_lase_scan > 5:
            self.cnt_lase_scan=0
            cnt_obstacle = 0
            # #test 激光雷达数据的各个点对应的角度
            # length=len(data.ranges)
            # print("length of lase data:", length) 
            # for tempi in range(0, length-1):
            #     if(data.ranges[tempi] > 0) and (data.ranges[tempi] < 0.40) :
            #         print("obstacle point:", tempi,data.ranges[tempi])


            for tempi in range(0, self.length_point-1):
                if data.ranges[tempi]>0:
                    #%算出三角形两边的夹角
                    angle_tf_point=self.start_angle_point_laser+math.pi+tempi*self.step_angle_point
                    if angle_tf_point>math.pi:
                        angle_tf_point = 2*math.pi - angle_tf_point
                    #%根据余弦定理，知道三角形的两条边长和其夹角，可以求出第三条边的长度：c^2=a^2+b^2-2*a*b*cosC
                    value_C_distance=math.sqrt(self.value_A*self.value_A+data.ranges[tempi]*data.ranges[tempi]-2*self.value_A*data.ranges[tempi]*math.cos(angle_tf_point))
                    if value_C_distance>(self.value_radius_car+0.02) and value_C_distance<(self.TH_value_obstacle+self.value_radius_car):
                        # print("obstacle point:", tempi)
                        cnt_obstacle +=1
                        if cnt_obstacle>0:
                            self.obstaclefront = True
                            self.obstaclefront_clear_cnt = 10 #清除前方有障碍物标志计数器
            if cnt_obstacle==0:
                if self.obstaclefront == True:
                    self.obstaclefront_clear_cnt -= 1 
                    if self.obstaclefront_clear_cnt <=0:
                        self.obstaclefront_clear_cnt =0
                        self.obstaclefront = False
            if self.obstaclefront!=self.obstaclefront_last:
                self.ob_obstaclefront_pub.publish(self.obstaclefront) #发送激光雷达探测前方障碍物距离话题
            self.obstaclefront_last=self.obstaclefront

    #更新recharge handle 状态值
    #指示充电步骤：0:not recharge 1:going to recharge point 2:matching the ir 3:start recharging
    def fun_update_recharge_handle(self,data_update):
      
        if data_update==0:
            self.in_recharing = 0 
            self.recharge_handle_pub.publish(data_update)
            # rospy.loginfo("interupt recharging!")
        elif data_update==1:
            self.in_recharing = 1 
            self.recharge_handle_pub.publish(data_update)
            # rospy.loginfo("moving to recharge point!")
        elif data_update==2:
            self.in_recharing = 2 
            self.recharge_handle_pub.publish(data_update)
            # rospy.loginfo("reach recharge point and start recharge") 
        elif data_update==3:
            if self.in_recharing !=3:
                self.in_recharing = 3
                self.ob_recharge_control_status_pub.publish(True)
                self.ob_location_status_pub.publish(True)#定位成功
                rospy.loginfo("recharger is charging!!")
                rospy.sleep(2)
                #等待机器人的异常状态已经解除了再发送充电状态更新
                self.recharge_handle_pub.publish(data_update)

    #机器人从充电桩上缓慢退出函数
    def fun_robot_leave_charger_control(self):
        # 向前移动退出充电桩

        #test code 暂时保留退出充电桩前使能
        self.ob_exit_recharge_pub.publish(1)#使能电机驻力
        rospy.sleep(0.5)
        
        # print("go forward!! ")
        twist = Twist()
        tb = rospy.get_time()
        delta_time = 0
        temp_time = 0
        temp_time_timeout = 0
        flag_obstaclefront_audio_play = 0
        while(temp_time < self.forward_time and temp_time_timeout < (self.forward_time*4) and (not rospy.is_shutdown()) and (self.ob_error_status == 0)):
            delta_time = rospy.get_time() - tb
            tb = rospy.get_time()
            temp_time_timeout += delta_time
            if self.obstaclefront is False:
                temp_time += delta_time
                twist.linear.x = self.forward_vel
            else:
                twist.linear.x = 0
                if (temp_time_timeout>=(self.forward_time*2)) and (flag_obstaclefront_audio_play == 0):
                    self.infovoicepub.publish("pathplan_fail") 
                    flag_obstaclefront_audio_play = 1                                                                                             
            self.cmd_vel_pub.publish(twist)
        
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def recharge_value_cb(self, data):
        self.car_voltage_value = copy.deepcopy(data.data)
        if self.car_voltage_value <= 453:
            self.flag_low_voltage = self.flag_low_voltage| 0x01
        else:
            self.flag_low_voltage = self.flag_low_voltage & 0xFE

        if self.ob_web_tool_runing_Flag==False and (self.flag_low_voltage&0X01):
            rospy.loginfo("low voltage!! auto recharge!!")
            tempbool = Bool()
            tempbool.data = True
            self.ob_auto_recharge_cb(tempbool)

    def recharge_value_top_cb(self, data):
        self.car_voltage_value_top = copy.deepcopy(data.data)
        if self.car_voltage_value_top <= 453:
            self.flag_low_voltage = self.flag_low_voltage| 0x02
        else:
            self.flag_low_voltage = self.flag_low_voltage & 0xFD

        if self.ob_web_tool_runing_Flag==False and (self.flag_low_voltage&0X02):
            rospy.loginfo("low voltage top!! auto recharge!!")
            tempbool = Bool()
            tempbool.data = True
            self.ob_auto_recharge_cb(tempbool)
 

    #获取底层充电状态
    #0: matching status
    #2: get the handshake signal
    #3: charging
    #4: charging and error 
    #5: battery is full and charge finish    
    def recharge_status_cb(self, data):
        self.recharge_status = data.data
        # print("self.recharge_status:", self.recharge_status)

    def recharge_status_top_cb(self, data):
        self.recharge_status_top = data.data
        # print("self.recharge_status_top:", self.recharge_status_top)

    def ir_data_cb(self, data):
        self.ir_data = data.data

        # if self.find_ir_init_flag is True and self.recharge_status == 0:
        #     print("self.ir_data",hex(self.ir_data))
        #     print("self.ir_data",self.ir_data/256)
        #     print("self.ir_data",self.ir_data&0xff)

    def ob_nav_finish_cb(self, data):
        
        #收到导航完成命令，判断是否导航到充电点
        if self.in_recharing == 1:
            self.move_to_charge_pos_finish_flag = True
            if data.data == 1:
                self.move_to_charge_pos_success_flag = True
            else:
                self.move_to_charge_pos_success_flag = False

    # 自动充电启动与取消回调函数
    def ob_auto_recharge_cb(self, data):
        
        rospy.loginfo("receive auto charge command: "+str(data.data))
        if(data.data is True):
            # rospy.loginfo("receive start auto charge command!")         
            if self.ob_error_status == 0:
                #如果机器人不是在派送、导航或者充电状态，则启动一次自动充电操作流程
                if self.recharge_step == 0:
                    self.recharge_step = 7#nav to the recharge pos
                    rospy.loginfo("recharge_step: "+str(self.recharge_step))
                    rospy.loginfo("start auto charge!")
                else:
                    rospy.loginfo("robot is charging status!!!!")
            else:
                rospy.loginfo("in delivery or error, cannot auto recharge, please wait!")
        else:
            if self.flag_low_voltage==0:#低电量不能退出自动充电
                if self.recharge_step == 5:#5: into recharging status and detect exit charging
                    self.recharge_step = 8#6: exit the charging status
                    rospy.loginfo("recharge_step: "+str(self.recharge_step))
                    # rospy.loginfo("exit auto charge!")
                else:
                    self.recharge_step = 0
                    self.flag_recharge_step_first_run = True
                    rospy.loginfo("recharge_step: "+str(self.recharge_step))

    # 设定当前位置为充电点
    def ob_rechargePose_current_cb(self, data):
        if(data.data is True):
            try:
            # if 1:
                poseoperation=PoseOperation()
                poseoperation.opt   = "update"
                poseoperation.id    = 100001
                
                try:
                    resp = self.posemanage(poseoperation)
                    # print(resp.status)
                    # print(resp.posedefine)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                
            except:
		        rospy.logerr("cannot get the transform from map to base_footprint!!")

    def mainloop(self):#默认调用时间间隔为0.1s

        if self.recharge_step == 7:
            self.recharge_step = 1#nav to the recharge pos
            self.flag_recharge_step_first_run = True
            self.recharge_try_cnt = 0 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
            rospy.loginfo("recharge_step: "+str(self.recharge_step))
        elif self.recharge_step == 8:
            self.recharge_step = 6#nav to the recharge pos
            self.flag_recharge_step_first_run = True
            rospy.loginfo("recharge_step: "+str(self.recharge_step))
        elif self.recharge_step == 0:#idle

            if self.flag_recharge_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_recharge_step_first_run = False
                #发送充电退出状态
                self.fun_update_recharge_handle(0)

            #判断机器人是否已经坐到了充电桩上或者到了启动自动充电时间点
            if self.recharge_status >= 3:
                rospy.loginfo("start into recharging!!!!")
                # 底层进入了正在底层充电状态，机器人已经在充电
                self.recharge_step = 4#4: detect recharge success
                self.flag_recharge_step_first_run = True
                rospy.loginfo("recharge_step: "+str(self.recharge_step))
            

            now = time.strftime("%H%M%S", time.localtime()) # 获取当前时间
            # 如果当前时间=设置的充电时间
            if self.ob_web_tool_runing_Flag==False and (now == self.chargetime) and (self.robot_for_guest_greeting == True or self.robot_for_meal_delivery == True):
                rospy.loginfo("charge on time!! auto recharge!!")
                tempbool = Bool()
                tempbool.data = True
                self.ob_auto_recharge_cb(tempbool)

        elif self.recharge_step == 1:#nav to the recharge pos
            if self.flag_recharge_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_recharge_step_first_run = False
                # 首先导航到充电点
                self.fun_update_recharge_handle(1)
                self.move_to_charge_pos_finish_flag = False
                self.move_to_charge_pos_success_flag = False
                tempInt32 = Int32()
                tempInt32.data = 100001
                self.ob_map_goal_id_pub.publish(tempInt32)

            if self.move_to_charge_pos_finish_flag == True:
                if self.move_to_charge_pos_success_flag == True:
                    # 成功导航到充电点
                    self.recharge_step = 2#2: find the ir data 
                    self.flag_recharge_step_first_run = True
                    rospy.loginfo("recharge_step: "+str(self.recharge_step))
                else:
                    self.recharge_step = 0#idle
                    self.flag_recharge_step_first_run = True
                    rospy.loginfo("recharge_step: "+str(self.recharge_step))
                    rospy.loginfo("can not reach recharge point: stop recharge!!")
            
            if self.ob_error_status != 0:
                self.recharge_step = 0#idle
                self.flag_recharge_step_first_run = True
                rospy.loginfo("recharge_step: "+str(self.recharge_step))

        elif self.recharge_step == 2:#find the ir data
            if self.flag_recharge_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_recharge_step_first_run = False
                self.lost_ir_cnt = 0
                self.fun_update_recharge_handle(2)
                self.ob_exit_recharge_pub.publish(0)#到达充电点开始匹配充电桩前要发送零，清零开始匹配标志位
                rospy.sleep(0.5)#等待机器人异常状态被发出
            #寻找红外信号,如果有红外信号则进入对齐充电桩操作
            # print("self.ir_data",hex(self.ir_data))
            # rospy.loginfo("self.ir_data:"+str(hex(self.ir_data)))
            if self.ir_data:
                rospy.loginfo("into matching charger state")   
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.recharge_step = 3#nav to the recharge pos
                self.flag_recharge_step_first_run = True
                rospy.loginfo("recharge_step: "+str(self.recharge_step))    
            else:
                self.lost_ir_cnt +=1
                if self.lost_ir_cnt>=120:#3s超时
                    #两边都没有找到红外信号,退出充电对齐模式，显示充电异常状态
                    rospy.loginfo("cannot find the recharger,error!!")
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)

                    self.recharge_try_cnt += 1 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
                    if self.recharge_try_cnt >=3:
                        self.recharge_try_cnt=0
                        self.recharge_step = 0
                        self.flag_recharge_step_first_run = True
                        rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                        self.ob_recharge_control_status_pub.publish(False)
                        rospy.sleep(2)#等待机器人异常状态被发出
                    else:
                        self.fun_robot_leave_charger_control()
                        self.recharge_step = 1#nav to the recharge pos
                        self.flag_recharge_step_first_run = True     
                        rospy.loginfo("recharge_step: "+str(self.recharge_step))  

                elif self.lost_ir_cnt>20 and self.lost_ir_cnt<60:#3s超时
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = -0.1
                    self.cmd_vel_pub.publish(twist)
                elif self.lost_ir_cnt>60 and self.lost_ir_cnt<140:#3s超时
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = 0.1
                    self.cmd_vel_pub.publish(twist)

            if self.ob_error_status != 0:
                self.recharge_step = 0#idle
                self.flag_recharge_step_first_run = True
                rospy.loginfo("recharge_step: "+str(self.recharge_step))

        elif self.recharge_step == 3:#3:matching the charger
            if self.flag_recharge_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_recharge_step_first_run = False
                self.get_ir_and_turning_back_cnt=0
                self.lost_ir_cnt=0
                self.wait_matching_cnt=0

            #开始执行后退对齐充电桩
            while((self.recharge_step == 3) and (self.ob_error_status == 0)):
                
                if self.recharge_status == 0:#还没有坐上充电桩
                    if self.ir_data == 0:
                        rospy.sleep(0.1)
                        # 没有红外信号，有可能机器人已经移动到很靠近充电桩位置导致红外信号被挡住或者充电口弹片被压下
                        rospy.loginfo("matching charger,cannot get ir_data!!")
                        if self.lost_ir_cnt < 60:
                            self.lost_ir_cnt += 1
                            if self.lost_ir_cnt == 30:
                                # 尝试后退一下，确认充电弹片被准确压下
                                rospy.loginfo("matching charger, cannot get ir_data,try move back!!")
                                twist = Twist()
                                twist.linear.x = -0.02
                                twist.angular.z = 0
                                ta = rospy.get_time()
                                tb = rospy.get_time()
                                while((tb-ta) < 0.5) and (not rospy.is_shutdown()):
                                    tb = rospy.get_time()
                                    self.cmd_vel_pub.publish(twist)
                        else:
                            # 持续6s都没有找到红外信号，充电异常
                            rospy.loginfo("cannot find the ir_data,recharge error!!")
                            self.recharge_try_cnt += 1 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
                            if self.recharge_try_cnt >=3:
                                self.recharge_try_cnt=0
                                self.recharge_step = 0
                                self.flag_recharge_step_first_run = True
                                rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                                self.ob_recharge_control_status_pub.publish(False)
                                rospy.sleep(2)#等待机器人异常状态被发出
                            else:
                                self.fun_robot_leave_charger_control()
                                self.recharge_step = 1#nav to the recharge pos
                                self.flag_recharge_step_first_run = True     
                                rospy.loginfo("recharge_step: "+str(self.recharge_step))
                        twist.linear.x = 0
                        twist.angular.z = 0
                        self.cmd_vel_pub.publish(twist)
                    else:#self.ir_data != 0:
                        # 有红外信号则按照红外信号查找速度表
                        rospy.sleep(0.05)#等待机器人异常状态被发出
                        self.lost_ir_cnt = 0 #重置没有找到红外的计数
                        left_ir_value = self.ir_data/256
                        if left_ir_value>7:
                            left_ir_value=7
                            rospy.loginfo("left_ir_data error")
                        right_ir_value = self.ir_data&0xff
                        if right_ir_value>7:
                            right_ir_value=7
                            rospy.loginfo("right_ir_data error")              
                        self.get_ir_and_turning_back_cnt+=1
                        if self.get_ir_and_turning_back_cnt>1200:#0.05*600=30s
                            twist.linear.x = 0
                            twist.angular.z = 0
                            self.cmd_vel_pub.publish(twist)
                            rospy.loginfo("matching too long,recharge error!!")
                            self.recharge_try_cnt += 1 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
                            if self.recharge_try_cnt >=3:
                                self.recharge_try_cnt=0
                                self.recharge_step = 0
                                self.flag_recharge_step_first_run = True
                                rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                                self.ob_recharge_control_status_pub.publish(False)
                                rospy.sleep(2)#等待机器人异常状态被发出
                            else:
                                self.fun_robot_leave_charger_control()
                                self.recharge_step = 1#nav to the recharge pos
                                self.flag_recharge_step_first_run = True     
                                rospy.loginfo("recharge_step: "+str(self.recharge_step))
                        else:

                            (_trans, _rot) = self.tflistener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
                            current_pos_x = _trans[0]
                            current_pos_y = _trans[1]
                            current_distance = sqrt(pow((current_pos_x - self.recharge_position.position.x), 2) + pow((current_pos_y - self.recharge_position.position.y), 2))

                            if current_distance > 0.25:
                                self.angular_vel = 0.2#红外对准时的角速度
                                self.linear_vel = -0.1 #红外对准时的后退速度
                            else:
                                self.angular_vel = 0.15
                                self.linear_vel = -0.02

                            # 根据红外得到的线速度表
                            self.vel_table = [
                            [self.linear_vel, self.linear_vel,	self.linear_vel,	self.linear_vel,	0,	self.linear_vel,	0,	self.linear_vel],
                            [0,	0,	0,	0,	0,	0,	0,	0],
                            [self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel],
                            [0,	0,	self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,],
                            [0,	self.linear_vel,	self.linear_vel,	0,	0,	0,	0,	0],
                            [self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	0,	self.linear_vel,	self.linear_vel,	self.linear_vel],
                            [self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel],
                            [0,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel,	self.linear_vel]]

                            # 根据红外得到的角速度表
                            self.angular_table = [
                            [0,	0,	0,	0,	-self.angular_vel,	0,	-self.angular_vel,	0],
                            [self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel,	self.angular_vel],
                            [0,	self.angular_vel,	0,	0,	0,	0,	0,	0],
                            [self.angular_vel,	self.angular_vel,	0,	self.angular_vel,	0,	0,	0,	0],
                            [-self.angular_vel,	0,	0,	-self.angular_vel,	-self.angular_vel,	-self.angular_vel,	-self.angular_vel,	-self.angular_vel],
                            [0,	self.angular_vel,	0,	0,	self.angular_vel,	0,	0,	0],
                            [self.angular_vel,	0,	0,	0,	0,	0,	-self.angular_vel,	0],
                            [self.angular_vel,	self.angular_vel,	0,	0,	0,	0,	0,	0]]

                            twist = Twist()
                            twist.linear.x = self.vel_table[left_ir_value][right_ir_value] #通过红外查找速度表
                            twist.angular.z = self.angular_table[left_ir_value][right_ir_value]
                            # 给定速度，时长50ms,后退对齐充电桩
                            self.cmd_vel_pub.publish(twist)

                elif self.recharge_status == 2:# 已经坐上充电桩，充电桩正在匹配
                    rospy.sleep(0.1)
                    self.wait_matching_cnt += 1
                    if self.wait_matching_cnt ==1:#后退0.5s，顶上充电桩
                        rospy.loginfo("self.recharge_status == 2,move back to lock the charger")
                        twist = Twist()
                        twist.linear.x = -0.05
                        twist.angular.z = 0
                        tb = rospy.get_time()
                        delta_time = 0
                        temp_time = 0
                        while((temp_time) < 0.1) and (not rospy.is_shutdown() and (self.recharge_status == 2)):
                            delta_time = rospy.get_time() - tb
                            tb = rospy.get_time()
                            temp_time += delta_time
                            self.cmd_vel_pub.publish(twist)
                        twist.linear.x = 0
                        twist.angular.z = 0
                        self.cmd_vel_pub.publish(twist)

                    elif self.wait_matching_cnt > 100:#等待10s，判断充电是否匹配成功，之前15s，时间太长
                        # 匹配超时，当作匹配失败,充电异常
                        rospy.loginfo("matching error,recharge error!!")
                        self.recharge_try_cnt += 1 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
                        if self.recharge_try_cnt >=3:
                            self.recharge_try_cnt=0
                            self.recharge_step = 0
                            self.flag_recharge_step_first_run = True
                            rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                            self.ob_recharge_control_status_pub.publish(False)
                            rospy.sleep(2)#等待机器人异常状态被发出
                        else:
                            self.fun_robot_leave_charger_control()
                            self.recharge_step = 1#nav to the recharge pos
                            self.flag_recharge_step_first_run = True     
                            rospy.loginfo("recharge_step: "+str(self.recharge_step))

                elif self.recharge_status == 3 or self.recharge_status == 5:
                    # 底层进入了正在底层充电状态，机器人已经在充电
                    self.recharge_step = 4#4:detect recharge success
                    self.flag_recharge_step_first_run = True
                    rospy.loginfo("recharge_step: "+str(self.recharge_step))
                elif(self.recharge_status == 6) or (self.recharge_status_top == 6):
                    rospy.loginfo("detect plugin two rechargers!!")
                    self.recharge_try_cnt += 1 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
                    if self.recharge_try_cnt >=3:
                        self.recharge_try_cnt=0
                        self.recharge_step = 0
                        self.flag_recharge_step_first_run = True
                        rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                        self.ob_recharge_control_status_pub.publish(False)
                        rospy.sleep(2)#等待机器人异常状态被发出
                    else:
                        self.fun_robot_leave_charger_control()
                        self.recharge_step = 1#nav to the recharge pos
                        self.flag_recharge_step_first_run = True     
                        rospy.loginfo("recharge_step: "+str(self.recharge_step))
            

            #退出对齐充电桩状态前先发送停止运动命令,保证机器人不会一直后退
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)

            if self.ob_error_status != 0:
                self.recharge_step = 0#idle
                self.flag_recharge_step_first_run = True
                rospy.loginfo("recharge_step: "+str(self.recharge_step))

        elif self.recharge_step == 4:#4:detect recharge success
            if self.flag_recharge_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_recharge_step_first_run = False
                self.wait_top_recharge_cnt=0
            
            if(self.recharge_status_top == 3) or (self.recharge_status_top == 5):
                # 判断上层电池正在充电或者已经充满
                rospy.loginfo("recharge success!!")
                self.recharge_step = 5#5: into recharging status and detect exit charging
                self.flag_recharge_step_first_run = True
                rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                    
            else:#(self.recharge_status_top != 3) and (self.recharge_status_top != 5):
                #上层电池没有在充电
                self.wait_top_recharge_cnt += 1
                if(self.wait_top_recharge_cnt > 5):#等待5秒都没有收到上层充电状态信号，则认为充电失败
                    rospy.loginfo("down is charging top is not charging !!")
                    self.recharge_try_cnt += 1 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
                    if self.recharge_try_cnt >=3:
                        self.recharge_try_cnt=0
                        self.recharge_step = 0
                        self.flag_recharge_step_first_run = True
                        rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                        self.ob_recharge_control_status_pub.publish(False)
                        rospy.sleep(2)#等待机器人异常状态被发出
                    else:
                        self.fun_robot_leave_charger_control()
                        self.recharge_step = 1#nav to the recharge pos
                        self.flag_recharge_step_first_run = True     
                        rospy.loginfo("recharge_step: "+str(self.recharge_step))
            #不能加异常判断,否则会反复进行充电/退出充电操作
            # if self.ob_error_status != 0:
            #     self.recharge_step = 0#idle
            #     self.flag_recharge_step_first_run = True
            #     rospy.loginfo("recharge_step: "+str(self.recharge_step))
            rospy.sleep(1)

        elif self.recharge_step == 5:#5: into recharging status and detect exit charging
            if self.flag_recharge_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_recharge_step_first_run = False
                #发送充电成功状态
                self.fun_update_recharge_handle(3)
                #切换地图到1楼,并设置机器人位置为开机点
                self.exp_floor = 1
                if self.currentfloor!=self.exp_floor:

                    self.ob_changing_current_map_pub.publish(True)
                    rospy.loginfo("+++++++++++++++++transform map+++++++++++++++++")       
                    self.reset_map = rospy.ServiceProxy('reset_map', reset_map)
                    # rospy.loginfo("ob_relocation.py wait for reset_map")
                    self.reset_map.wait_for_service()
                    # rospy.loginfo('found "reset_map" service')
                    floor_test_src = "/home/ob/.ob/map/OB_floor_" + str(self.exp_floor) + ".yaml"  #目标楼层
                    tempresult = self.reset_map(floor_test_src, "map")
                    floor_test_src = "/home/ob/.ob/map/OB_floor_location_" + str(self.exp_floor) + ".yaml"  #目标楼层
                    tempresult = self.reset_map(floor_test_src, "map_amcl")
                    if tempresult.result is True:
                        self.ob_current_floor_pub.publish(self.exp_floor)
                    rospy.sleep(3)
                    #等待地图切换完成后才能重定位

                #获取开机点坐标
                poseoperation=PoseOperation()
                poseoperation.opt   = "select"
                poseoperation.id    = 100000
                        
                resp = self.posemanage(poseoperation)

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
                rospy.sleep(1)
                self.ob_changing_current_map_pub.publish(False)

                # #启动一次YS重定位操作
                # #先求出方位角
                # _rot = [resp.posedefine.pose.orientation.x, resp.posedefine.pose.orientation.y, resp.posedefine.pose.orientation.z, resp.posedefine.pose.orientation.w]
                # self.yaw_cur = tf.transformations.euler_from_quaternion(_rot)[2]
                # self.yaw_cur = self.yaw_cur/math.pi*180 #弧度转角度

                # self.ob_localization = rospy.ServiceProxy('initial_pose_one', localization_srv)
                # rospy.loginfo("ob_recharge.py wait for ob_localization")
                # self.ob_localization.wait_for_service()
                # rospy.loginfo('found "ob_localization" service')
                # self.ob_changing_current_map_pub.publish(True)
                # # rospy.sleep(0.3)
                # rospy.loginfo('start localization!!')
                # tempresult = self.ob_localization(resp.posedefine.pose.position.x,resp.posedefine.pose.position.y,self.yaw_cur)
                # if tempresult.result is True:
                #     rospy.loginfo("ob_localization finish,score："+ str(tempresult.score))
                # self.ob_changing_current_map_pub.publish(False)

                rospy.loginfo("charge success,relocation finished")


            if self.recharge_status <3 or self.recharge_status_top <3:
                rospy.loginfo("recharg interrupt,error!!")
                self.recharge_try_cnt += 1 #尝试对齐充电次数记录,目前开放可以尝试对齐3次
                if self.recharge_try_cnt >=3:
                    self.recharge_try_cnt=0
                    self.recharge_step = 0
                    self.flag_recharge_step_first_run = True
                    rospy.loginfo("recharge_step: "+str(self.recharge_step)) 
                    self.ob_recharge_control_status_pub.publish(False)
                    rospy.sleep(2)#等待机器人异常状态被发出
                else:
                    self.fun_robot_leave_charger_control()
                    self.recharge_step = 1#nav to the recharge pos
                    self.flag_recharge_step_first_run = True     
                    rospy.loginfo("recharge_step: "+str(self.recharge_step))

            # elif self.recharge_status == 5 and self.recharge_status_top == 5:
                # rospy.loginfo(" all battery is full!!!!")

            now = time.strftime("%H%M%S", time.localtime()) # 获取当前时间
            # 如果当前时间=设置的退出充电时间
            if self.ob_web_tool_runing_Flag==False and (now == self.exit_chargetime) and (self.robot_for_guest_greeting == True or self.robot_for_meal_delivery == True):

                if self.robot_for_guest_greeting == True:
                    #处于迎宾模式下才能满电退出充电桩,导航到迎宾点准备迎宾
                    self.ob_map_goal_id_outcharge = 100003
                elif self.robot_for_meal_delivery == True:
                    #处于送餐模式下满电退出充电桩,导航到原点准备派送
                    self.ob_map_goal_id_outcharge = 100002             
                tempbool = Bool()
                tempbool.data = False
                self.ob_auto_recharge_cb(tempbool)
                rospy.loginfo("exit charge on time!! exit recharge!!")

            #不能加异常判断,否则会反复进行充电/退出充电操作
            # if self.ob_error_status != 0:
            #     self.recharge_step = 0#idle
            #     self.flag_recharge_step_first_run = True
            #     rospy.loginfo("recharge_step: "+str(self.recharge_step))
            rospy.sleep(1)
        elif self.recharge_step == 6:#6:exit the charging status
            if self.flag_recharge_step_first_run == True:#导航阶段第一次进入标识符,用于执行一些只需要运行一次的操作
                self.flag_recharge_step_first_run = False
  
            #清充电状态
            self.fun_update_recharge_handle(0)

            #执行在充电桩上退出自动充电操作.先往前走一段距离,大约到充电点位置,然后导航到目标点
            rospy.loginfo("stop charging go forward!!")
            self.fun_robot_leave_charger_control()
            
            rospy.loginfo("send map goal id ")           
            if self.ob_error_status == 0:
                tempInt32 = Int32()
                tempInt32.data = self.ob_map_goal_id_outcharge#导航到目标点
                self.ob_map_goal_id_pub.publish(tempInt32)
                # rospy.loginfo("stop charging finish go to the goal pos!!")
            while(self.recharge_status>0 and self.ob_error_status == 0):                
                rospy.sleep(1)
            self.recharge_step = 0
            self.flag_recharge_step_first_run = True
            rospy.loginfo("recharge_step: "+str(self.recharge_step))


    def shutdown(self):
        rospy.loginfo("Exit Auto Recharge")
        rospy.sleep(1)



if __name__ == '__main__':

    rospy.init_node('ob_auto_recharge', anonymous=True)
    recharger = Recharge()
    while(not rospy.is_shutdown()):
        if recharger.relocationfinish_flag is True:
            recharger.mainloop() # 充电程序的主循环
            rospy.sleep(0.1)
    # rospy.spin()

