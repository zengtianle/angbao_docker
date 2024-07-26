#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
import binascii
import sys
import rospy
import math
import copy

from std_msgs.msg import Float32, Int16

class ServoControl():
    def __init__(self):

        rospy.loginfo("......ob_servo_conttol.py is running.....")
        rospy.on_shutdown(self.shutdown)

        self.ser = serial.Serial('/dev/ob_servo', 115200, timeout=0.5) #端口与波特率
        self.ob_welcome_flag = 0
        self.ob_welcome_cnt = 0

        self.servo_id = -1  #舵机的id
        self.servo_mode = -1 #舵机运行的模式
        self.control_mode = 0 # 0: back center; 1: turning around; 2: stop servo; 3: tracking mode 4: wait mode
        self.control_mode_last = 0 # 0: back center; 1: turning around; 2: stop servo; 3: tracking mode 4: wait mode
        self.servo_position = 1500
        self.servo_position_last = 1500    
        self.desired_position = 1500
        self.value_detect_position = 0        
        self.face_position_x = 0
        self.face_position_y = 0
        self.face_update_flag = False # update when topic isn't 0
        self.face_lost_cnt = 0
        self.timer_cnt_mode0 = 0
        self.turn_around_step = 0 #没有找到人脸，机器人转头的步骤
        self.turn_around_time = 4000 #6000 #0~9999 # unit: ms for 1000 pwm 设置主段1000pwm的时间，单位：ms
        self.pwm_min = 500 # 500 for minimum 舵机最小的pwm
        self.pwm_max = 2500 # 2500 for maximum #舵机最大的pwm

        self.angle_torlence = 30 # unit: pwm  判断是否到达目标位置的阈值

        self.angle_torlence_face = 100 # unit: pixel 判断人脸移动的阈值，单位：像素
        self.Kp_servo_control=-0.6 #-1.13  #控制偏差的比例，类似于kp


        rospy.Subscriber('ob_face_position', Float32, self.ob_face_position_cb) #zark
        rospy.Subscriber("ob_welcome", Int16, self.ob_welcome_cb)
        rospy.Subscriber("ob_set_servo_turn_around_time", Int16, self.ob_set_servo_turn_around_time_cb)
        self.ob_servo_turning_pub = rospy.Publisher("ob_servo_turning", Int16, queue_size = 5)

        rospy.loginfo("......wait_for_data.....")
        rospy.sleep(3)


    def ob_set_servo_turn_around_time_cb(self, data):
        self.turn_around_time=data.data

    def ob_face_position_cb(self, data):
        # print data.data
        if data.data == -2:
            # 正在对话，暂停转动
            if self.control_mode != 4:
                self.control_mode_last = copy.deepcopy(self.control_mode)
                self.control_mode = 4
                # print "self.control_mode_last", self.control_mode_last, "self.control_mode", self.control_mode
        # elif data.data == -3:
        #     if self.control_mode != 0:
        #         self.control_mode_last = copy.deepcopy(self.control_mode)
        #         self.control_mode = 0
        #         self.timer_cnt_mode0 = 0
        #         print "self.control_mode_last", self.control_mode_last, "self.control_mode", self.control_mode
        else:
            if self.control_mode == 4:
                self.control_mode = copy.deepcopy(self.control_mode_last)
                self.control_mode_last = 4
                # print "self.control_mode_last", self.control_mode_last, "self.control_mode", self.control_mode
            if data.data == -1:
                # 没找到人脸
                if(self.control_mode == 3):
                    self.face_lost_cnt += 1
                    if(self.face_lost_cnt > 30):
                        self.face_lost_cnt = 0
                        self.control_mode = 0
                        self.timer_cnt_mode0 = 0
            else:
                # 找到人脸，获取人脸坐标
                self.face_lost_cnt = 0
                self.face_update_flag = True
                self.face_position_x = int(data.data)
                self.face_position_y = math.modf(data.data)
                if(self.control_mode <= 1):
                    self.control_mode = 2
                    self.timer_cnt = 0
        
    def ob_welcome_cb(self, data):
        self.ob_welcome_flag = data.data
        if(data.data == 0):
            self.ob_welcome_cnt = 0

        elif(data.data == 1):
            self.timer_cnt_mode0 = 0
            self.face_position_x = 0
            self.face_position_y = 0
            self.face_update_flag = False # update when topic isn't 0
            self.timer_cnt_mode0 = 0
            self.turn_around_step = 0
            

    def get_servo_id(self):
        if not self.ser.isOpen():
            self.ser.open()
        cmd_str = "#255PID!"
        n = self.ser.write(cmd_str)
        time.sleep(0.1)
        n = self.ser.inWaiting()
        # print n
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            self.servo_id = rec_hex[1:4]
            print "Servo ID: " + (self.servo_id)
        # else:
        #     print "receive nothing!!"
        
    def get_servo_mode(self):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+str(self.servo_id)+"PMOD!"
        n = self.ser.write(cmd_str)
        time.sleep(0.1)
        n = self.ser.inWaiting()
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            self.servo_mode = rec_hex[rec_hex.find('!')-1]
            print "Servo MODE: " + (self.servo_mode)
        # else:
        #     print "receive nothing!!"

    def get_servo_position(self):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+str(self.servo_id)+"PRAD!"
        n = self.ser.write(cmd_str)
        time.sleep(0.01)
        # print n
        n = self.ser.inWaiting()
        # print n
        if n > 0:
            rec_hex = self.ser.read(n)
            # print "receive:" + rec_hex
            temp_ser_pos = rec_hex[5:9]
            try:
                
                if abs(int(temp_ser_pos)-self.servo_position_last)>=100:#去掉异常值
                    self.servo_position = self.servo_position_last
                else:
                    self.servo_position = int(temp_ser_pos)
                self.servo_position_last = self.servo_position
            except:
                print "usart data error"
            # print "Servo POSITION: " + str(self.servo_position)
        # else:
        #     print "receive nothing!!"

    # 设置舵机的工作模式
    def set_servo_mode(self, mode):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+(self.servo_id)+"PMOD" + str(mode) + "!"
        n = self.ser.write(cmd_str)
        # print n
        time.sleep(0.1)
        n = self.ser.inWaiting()
        # print n
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            if 'OK' in rec_hex:
                print "Set <servo mode> successfully!!"
            else:
                print "Set <servo mode> Fail!!"
        # else:
        #     print "receive nothing!!"

    # 舵机停止工作
    def servo_stop(self):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+(self.servo_id)+"PDST!"
        n = self.ser.write(cmd_str)
        # print n
        time.sleep(0.1)
        n = self.ser.inWaiting()
        # print n
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            if 'OK' in rec_hex:
                print "Set <servo stop> successfully!!"
            else:
                print "Set <servo stop> Fail!!"
        # else:
        #     print "receive nothing!!"

    def servo_continue(self):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+(self.servo_id)+"PDCT!"
        n = self.ser.write(cmd_str)
        time.sleep(0.1)
        n = self.ser.inWaiting()
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            if 'OK' in rec_hex:
                print "Set <servo continue> successfully!!"
            else:
                print "Set <servo continue> Fail!!"
        # else:
        #     print "receive nothing!!"

    def servo_pause(self):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+(self.servo_id)+"PDPT!"
        n = self.ser.write(cmd_str)
        time.sleep(0.1)
        n = self.ser.inWaiting()
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            if 'OK' in rec_hex:
                print "Set <servo pause> successfully!!"
            else:
                print "Set <servo pause> Fail!!"
        # else:
        #     print "receive nothing!!"

    # 舵机释放力矩
    def servo_release_torque(self):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+(self.servo_id)+"PCSM!"
        n = self.ser.write(cmd_str)
        time.sleep(0.1)
        n = self.ser.inWaiting()
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            if 'OK' in rec_hex:
                print "Set <Servo release torque> successfully!!"
            else:
                print "Set <Servo release torque> Fail!!"
        # else:
        #     print "receive nothing!!"

    # 舵机恢复力矩
    def servo_recover_torque(self):
        if self.servo_id == -1:
            print "No device"
            return
        cmd_str = "#"+(self.servo_id)+"PCSR!"
        n = self.ser.write(cmd_str)
        time.sleep(0.1)
        n = self.ser.inWaiting()
        if n > 0:
            rec_hex = self.ser.read(n)
            print "receive:" + rec_hex
            if 'OK' in rec_hex:
                print "Set <Servo recover Torque> successfully!!"
            else:
                print "Set <Servo recover Torque> Fail!!"
        # else:
        #     print "receive nothing!!"

    # 控制舵机转动角度，angle转动的目标角度，settime所需时间，单位ms
    def servo_control(self, angle, settime):
        if self.servo_id == -1:
            print "No device"
            return
        
        # the sevo maximun and minimun, cannot be changed
        if(angle > self.pwm_max):
            angle = self.pwm_max
        elif (angle < self.pwm_min):
            angle = self.pwm_min
        angle = int(angle)
        self.desired_position=angle
        if(settime > 9999):
            settime = 9999
        settime = int(settime)

        cmd_str = "#"+(self.servo_id)+"P"+(str(angle).zfill(4))+"T" + str(settime).zfill(4) +"!"
        # print cmd_str
        n = self.ser.write(cmd_str)

        time.sleep(0.01)

    def mainloop(self):
        '''
        0.05s period
        '''
        if(self.ob_welcome_flag == 0):
            if(self.ob_welcome_cnt == 0):
                self.ob_welcome_cnt = 1
                self.get_servo_position()
                temp_x = (1500 - self.servo_position)
                temp_time = abs(temp_x) * self.turn_around_time / 1000.0
                self.servo_control(1500, temp_time)

        elif(self.ob_welcome_flag == 1):
            if self.control_mode == 0:
                # 舵机返回中间
                if(self.timer_cnt_mode0 == 0):
                    self.timer_cnt_mode0 = 1
                    # self.servo_control(1500, self.turn_around_time)
                    self.get_servo_position()
                    temp_x = (1500 - self.servo_position)
                    temp_time = abs(temp_x) * self.turn_around_time / 1000.0
                    self.servo_control(1500, temp_time)

                # #不断判断舵机是否返回中间，如果返回完成则进入循环转头寻找人脸模式
                # self.get_servo_position()
                # if abs(self.servo_position - 1500) <= (self.angle_torlence / 2):
                #     self.control_mode = 1
                #     self.turn_around_step = 0
                rospy.sleep(0.1)

            elif self.control_mode == 1:
                # 没有找到人脸，机器人转头
                if self.control_mode_last == 4:
                    # 如果上一个状态时对话状态，那么恢复之前的状态
                    self.control_mode_last = 1
                    if(self.turn_around_step % 2 == 1):
                        # 如果之前的步骤序号为基数，则回退到上一步
                        self.turn_around_step -= 1

                if(self.turn_around_step == 0):
                    # 舵机转向最左
                    self.servo_control(self.pwm_min, self.turn_around_time)
                    self.turn_around_step = 1
                
                elif (self.turn_around_step == 1):
                    # 判断舵机是否已经转向最左
                    self.get_servo_position()
                    if abs(self.servo_position - self.pwm_min) <= self.angle_torlence:
                        self.turn_around_step = 2

                elif(self.turn_around_step == 2):
                    # 舵机回中
                    self.servo_control(1500, self.turn_around_time)
                    self.turn_around_step = 3

                elif (self.turn_around_step == 3):
                    # 判断舵机是否已经回中
                    self.get_servo_position()
                    if abs(self.servo_position - 1500) <= self.angle_torlence:
                        self.turn_around_step = 4

                elif(self.turn_around_step == 4):
                    # 舵机转到最右
                    self.servo_control(self.pwm_max, self.turn_around_time)
                    self.turn_around_step = 5

                elif (self.turn_around_step == 5):
                    # 判断舵机是否已经转到最右
                    self.get_servo_position()
                    if abs(self.servo_position - self.pwm_max) <= self.angle_torlence:
                        self.turn_around_step = 6

                elif(self.turn_around_step == 6):
                    # 舵机回中
                    self.servo_control(1500, self.turn_around_time)
                    self.turn_around_step = 7

                elif (self.turn_around_step == 7):
                    # 判断舵机是否已经回中
                    self.get_servo_position()
                    if abs(self.servo_position - 1500) <= self.angle_torlence:
                        self.turn_around_step = 8

                elif(self.turn_around_step == 8):
                    # 舵机重复整个流程
                    self.turn_around_step = 0

                rospy.sleep(0.1)
                # print "self.control_mode", self.control_mode, "self.turn_around_step", self.turn_around_step

            elif self.control_mode == 2:
                # 舵机停止转动
                self.servo_stop()
                self.control_mode = 3
                rospy.sleep(0.1)

            elif self.control_mode == 3:
                # 人脸跟踪
                if self.face_update_flag is True:
                    self.face_update_flag = False
                    temp_x = self.face_position_x - 320 #人脸偏离中心点的像素
                    if abs(temp_x) >= self.angle_torlence_face: #如果偏差大于阈值
                        # print "temp_x", temp_x
                        temp_x *= self.Kp_servo_control  #控制偏差的比例，类似于kp
                        # print "temp_x", temp_x
                        temp_time = abs(temp_x) * 1.8 #控制舵机转动的时间
                        if temp_time < 500: #如果时间小于0.5s，则设置为0.5s
                            temp_time = 500
                        self.get_servo_position()#获取当前舵机的角度
                        self.desired_position = temp_x + (self.servo_position) #设置舵机的目标角度
                        # print "self.desired_position: ", self.desired_position," temp_x: ", temp_x," self.servo_position: ", self.servo_position
                        self.servo_control(self.desired_position, temp_time)
            elif self.control_mode == 4:
                # stop servo and wait
                self.servo_stop()
                rospy.sleep(1)
                    
            self.get_servo_position()
            temp_angle=self.servo_position - self.desired_position
            if abs(temp_angle)<= self.angle_torlence:
                temp_angle=0
            else:
                temp_angle=1

            if temp_angle!=self.value_detect_position:#有变化时才发布话题
                self.ob_servo_turning_pub.publish(int(temp_angle))
                rospy.loginfo("Servo Control is change!!"+ str(temp_angle))
            self.value_detect_position=temp_angle

    def shutdown(self):
        rospy.loginfo("Exit Servo Control!!")
        self.servo_control(1500, 9999)
        rospy.loginfo("Reset Servo!!")
        rospy.sleep(0.5)
        self.ser.close()
        rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node('ob_servo_control', anonymous=True)
    servo = ServoControl()
    servo.get_servo_id() #获取舵机id
    servo.get_servo_mode() #获取舵机当前工作模式
    if servo.servo_mode != "3":
        servo.set_servo_mode(3) #设置工作模式为3

    while not rospy.is_shutdown():
        servo.mainloop()
        rospy.sleep(0.05)
    rospy.spin()