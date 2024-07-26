#!/usr/bin/env python
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist
import os, time
import thread

import math
from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial
import serial.tools.list_ports
import roslib

from geometry_msgs.msg import Quaternion, Twist, Pose ,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16, Int32, UInt16, Float32, String, Byte
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Range, Imu, LaserScan
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField 
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped, Pose, PoseArray
import struct
import binascii

import copy
 
ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


SERVO_MAX = 180
SERVO_MIN = 0



class ObRobot:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12


    
    def __init__(self, port="/dev/ttyUSB0", baudrate=230400, timeout=0.5):
        
        self.port_init = port
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

        self.WAITING_HEADER0_RECIEVE = 0
        self.WAITING_HEADER1_RECIEVE = 1
        self.RECEIVE_LEN = 2
        self.RECEIVE_PACKAGE = 3
        self.RECEIVE_CHECK = 4
        self.HEADER0_SEND = 0xff
        self.HEADER1_SEND = 0xaa
        
        self.SUCCESS = 0
        self.FAIL = -1

        self.receive_state_ = self.WAITING_HEADER0_RECIEVE
        self.receive_check_sum_ = 0
        self.payload_command = ''
        self.payload_ack = ''
        self.payload_args = ''
        self.payload_len = 0
        self.byte_count_ = 0
        self.receive_message_length_ = 0
        self.reconnect_cnt = 0
        self.check_sum_result = False

        self.sendcmdad = 0 #zark

        self.pluse_or_velocity = 0 # 0 for pluse; 1 for velocity
        self.send_time = 0 # 0 for mcu not sending time; 1 for mcu sending time
    
        # Keep things thread safe
        self.mutex = thread.allocate_lock()
            
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):

        port_list = list(serial.tools.list_ports.comports())
        count_port = len(port_list)
        #print "port list length",count_port
        if count_port<=0:
            print "the serial port can not find!"
            os._exit(1)
        else:

            while(count_port>0):
                port_info = list(port_list[count_port-1])
                self.port = port_info[0]
                #print self.port
                self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
                print "check which port was connected to mcu>",self.port.name
                time.sleep(1)
                state_, val = self.get_baud()
                print "get_baud:", val
                if state_== self.SUCCESS and val == self.baudrate:
                    print "Connected at ", self.port.name," and baudrate is ",self.baudrate
                    print "OB car is ready."
                    break
                # else:
                #     print "Check is fail"
                self.port.close()
                count_port = count_port - 1
            if count_port==0:
                print "Cannot connect to OB car!"
                os._exit(1)

    # def connect(self):
    #     try:
    #         print "Connecting to OB car on port", self.port_init, "..."
    #         self.port = Serial(port=self.port_init, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
    #         # The next line is necessary to give the firmware time to wake up.
    #         time.sleep(1)
    #         state_, val = self.get_baud() #travis
    #         #state_, val = 115200 #travis
    #         if val != self.baudrate:
    #             time.sleep(1)
    #             state_, val  = self.get_baud()   
    #             if val != self.baudrate:
    #                 raise SerialException
    #         print "Connected at", self.baudrate
    #         print "OB car is ready."

    #     except SerialException:
    #         print "Serial Exception:"
    #         print sys.exc_info()
    #         print "Traceback follows:"
    #         traceback.print_exc(file=sys.stdout)
    #         print "Cannot connect to OB car!"
    #         # print "travis_debug", self.baudrate, self.get_baud(), "..." #travis
    #         # os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd)


    def receiveFiniteStates(self, rx_data):
        if self.receive_state_ == self.WAITING_HEADER0_RECIEVE:
            if rx_data == '\xff':
                self.receive_state_ = self.WAITING_HEADER1_RECIEVE
                self.receive_check_sum_ =0
                self.receive_message_length_ = 0
                self.byte_count_=0
                self.payload_ack = ''
                self.payload_args = ''
                self.payload_len = 0
                self.check_sum_result = False


        elif self.receive_state_ == self.WAITING_HEADER1_RECIEVE :
             if rx_data == '\xaa':
                 self.receive_state_ = self.RECEIVE_LEN
                 self.receive_check_sum_ = 0
             else:
                 self.receive_state_ = self.WAITING_HEADER0_RECIEVE

        elif self.receive_state_ == self.RECEIVE_LEN:
             self.receive_message_length_, = struct.unpack("B",rx_data)
             self.receive_state_ = self.RECEIVE_PACKAGE
             self.receive_check_sum_ = self.receive_message_length_
            #  print "RECEIVE_LEN: "
            #  print self.receive_message_length_
             self.payload_ack = ''
             self.payload_args = ''
             self.payload_len = 0
             self.byte_count_ = 0
        elif self.receive_state_ == self.RECEIVE_PACKAGE:
             if self.byte_count_==0:
                 self.payload_ack = rx_data
             else:
                 self.payload_args += rx_data
             uc_tmp_, = struct.unpack("B",rx_data)
             self.receive_check_sum_ = self.receive_check_sum_ + uc_tmp_
            #  print "RECEIVE_PACKAGE: "
            #  print uc_tmp_
             self.byte_count_ +=1
             if self.byte_count_ >= self.receive_message_length_:
                 self.receive_state_ = self.RECEIVE_CHECK

        elif self.receive_state_ == self.RECEIVE_CHECK:
            self.receive_state_ = self.WAITING_HEADER0_RECIEVE
            uc_tmp_, = struct.unpack("B",rx_data)
            # print "RECEIVE_CHECK: "
            # print uc_tmp_
            if uc_tmp_ == (self.receive_check_sum_)%255:
                self.check_sum_result = True      
                return 1
            else:
                self.check_sum_result = False
                print "checksum error" 
                return 2
        else:
            self.receive_state_ = self.WAITING_HEADER0_RECIEVE
        return 0

    def recv(self, timeout=1.0):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Stm32
        '''

        tb = rospy.get_time()
        delta_time = 0
        temp_time_timeout = 0
        c = ''
        value = ''
        attempts = 0
        c = self.port.read(1)
        while(self.receiveFiniteStates(c) == 0) and (temp_time_timeout < timeout):
            delta_time = rospy.get_time() - tb
            tb = rospy.get_time()
            temp_time_timeout += delta_time
            c = self.port.read(1)
            # attempts += 1
            # if attempts * self.interCharTimeout > timeout:
            #     return 0
        if temp_time_timeout >= timeout:
            print "port respond timeout: "
            return 0
        else:
            return 1

    # def recv(self, timeout=0.5):
    #     timeout = min(timeout, self.timeout)
    #     ''' This command should not be used on its own: it is called by the execute commands   
    #         below in a thread safe manner.  Note: we use read() instead of readline() since
    #         readline() tends to return garbage characters from the ObRobot
    #     '''
    #     c = ''
    #     value = ''
    #     attempts = 0
    #     c = self.port.read(1)
    #     #print str(binascii.b2a_hex(c))
    #     while self.receiveFiniteStates(c) != 1:
    #         c = self.port.read(1)
    #         #print str(binascii.b2a_hex(c))
    #         attempts += 1
    #         if attempts * self.interCharTimeout > timeout:
    #             return 0
    #     return 1
            
    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the ObRobot returning a single integer value.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0

        try:
            self.port.write(cmd)
            # print "write command: " + str(binascii.b2a_hex(cmd))
            res = self.recv(self.timeout)
            while(attempts < ntries and res ==0):
                try:
                    #重发一次
                    self.port.flushInput()
                    self.port.write(cmd)
                    # print "write command: " + str(binascii.b2a_hex(cmd))
                    self.receive_state_ = self.WAITING_HEADER0_RECIEVE
                    self.receive_check_sum_ = 0
                    self.payload_command = ''
                    self.payload_ack = ''
                    self.payload_args = ''
                    self.payload_len = 0
                    self.byte_count_ = 0
                    self.check_sum_result = False
                    res = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + str(binascii.b2a_hex(cmd))
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command 2: " + str(binascii.b2a_hex(cmd))
            return 0

        self.mutex.release()
        return 1


    # def execute(self, cmd):
    #     ''' Thread safe execution of "cmd" on the ObRobot returning a single integer value.
    #     '''
    #     self.mutex.acquire()
        
    #     try:
    #         self.port.flushInput()
    #     except:
    #         pass
        
    #     ntries = 1
    #     attempts = 0
        
    #     try:
    #     # if 1:
    #         self.port.write(cmd)
    #         res = self.recv(self.timeout)
    #         self.reconnect_cnt = 0
    #         # print("res:",res)
    #         # print "response : " + str(binascii.b2a_hex(res))
    #         while attempts < ntries and res !=1 :
    #             try:
    #                 self.port.flushInput()
    #                 self.port.write(cmd)
    #                 self.receive_state_ = self.WAITING_HEADER0_RECIEVE
    #                 self.receive_check_sum_ = 0
    #                 self.payload_command = ''
    #                 self.payload_ack = ''
    #                 self.payload_args = ''
    #                 self.payload_len = 0
    #                 self.byte_count_ = 0
    #                 self.check_sum_result = False
    #                 res = self.recv(self.timeout)
    #                 print "response : " + str(binascii.b2a_hex(res))
    #             except:
    #                 print "Exception executing command: " + str(binascii.b2a_hex(cmd))
    #             attempts += 1
    #     except:
    #         self.mutex.release()
    #         self.reconnect_cnt += 1
    #         if self.reconnect_cnt >=2:
    #             self.reconnect_cnt = 0
    #             # print "Exception executing command: " + str(binascii.b2a_hex(cmd))
    #             print "usart cannot write command"
    #             print "try close and reconnect"
    #             try:
    #                 # self.port.open()
    #                 # self.close()
    #                 # print "close"
    #                 self.connect()
    #                 print "reconnected!!"
    #             except:
    #                 print "cannot reconnected"

            
    #         return 0
        
    #     self.mutex.release()
    #     return 1
                                 

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x00) + struct.pack("B", 0x01)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           val, = struct.unpack('I', self.payload_args) #travis
           return self.SUCCESS, val 
        else:
           return self.FAIL, 0

    def set_unlock_locker(self):
        ''' Unlock the locker
        '''
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x18) + struct.pack("B", 0x19)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           return  self.SUCCESS
        else:
           return self.FAIL

    def set_disable_driver(self):
        ''' disable the motor driver
        '''
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x22) + struct.pack("B", 0x23)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           return  self.SUCCESS
        else:
           return self.FAIL


    def get_encoder_counts(self):
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
            enc_left_ob, enc_right_ob, yawangle_ob, speed_left_ob, speed_right_ob, delta_time_ob = struct.unpack('6h', self.payload_args) #zark
            # if yawangle_ob > 32768:
            #     yawangle_ob -=65536
            yawangle_ob = yawangle_ob / 100.0
            # print("voltage", voltage_ob)
            #print("getencoder:", enc_left_ob, enc_right_ob, yawangle_ob, whetherreach_ob, voltage_ob, recharge_ir_data_ob)
            return  self.SUCCESS, enc_left_ob, enc_right_ob, yawangle_ob, speed_left_ob, speed_right_ob, delta_time_ob
        else:
            #print("getencoder  error!!!")
            return self.FAIL, 0, 0, 0, 0, 0, 0


    def get_emergency_button(self):
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x15) + struct.pack("B", 0x16)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           emergency_state, _, = struct.unpack('2H', self.payload_args)
           return  self.SUCCESS, emergency_state
        else:
           return self.FAIL, 0

    def reset_encoders(self):
        # rospy.loginfo("reset encoder!!!")
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x03) + struct.pack("B", 0x04)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           return  self.SUCCESS
        else:
           return self.FAIL

    def exit_recharge(self,data):
        data_temp = Byte()
        if data.data == 1:
            data_temp.data = 1
        elif data.data == 0:
            data_temp.data = 0

        self.check_list = [0x02,0x1a, data_temp.data]
        self.check_num = self.get_check_sum(self.check_list)
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x02, 0x1a) + struct.pack("B", data_temp.data) + struct.pack("B", self.check_num)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_check_sum(self,list):
        list_len = len(list)
        cs = 0
        for i in range(list_len):
            #print i, list[i]
            cs += list[i]
        cs=cs%255
        return cs

    def drive(self, left, right):
        data1 = struct.pack("h", left)
        d1, d2 = struct.unpack("BB", data1)

        data2 = struct.pack("h", right)
        c1, c2 = struct.unpack("BB", data2)

        self.check_list = [0x05,0x04, d1, d2, c1, c2]
        
        self.check_num = self.get_check_sum(self.check_list)
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x05, 0x04) + struct.pack("hh", left, right) + struct.pack("B", self.check_num)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           return  self.SUCCESS
        else:
           return self.FAIL
        
    def stop(self):
        ''' Stop both motors.
        '''
        
        # self.drive(0, 0)
        temp_send_try_cnt = 0
        while(self.drive(0, 0)==self.FAIL and temp_send_try_cnt<5):
            temp_send_try_cnt+=1
            time.sleep(0.1)
        # if temp_send_try_cnt<5:
        #     rospy.loginfo("drive success")
        # else:
        #     rospy.loginfo("drive fail")  

    def get_firmware_version(self):
        ''' Get the current version of the firmware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x01) + struct.pack("B", 0x02)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
            robot_type,bigversion,smallversion0,smallversion1 = struct.unpack('BBBB', self.payload_args)
            if smallversion0 == 1:
                self.pluse_or_velocity = 0
            elif smallversion0 == 2:
                self.pluse_or_velocity = 1
            elif smallversion0 == 3:
                self.pluse_or_velocity = 2
            else:
                self.pluse_or_velocity = 10

            if smallversion1 == 1:
                self.send_time = 1
            else:
                self.send_time = 0

            return  self.SUCCESS, robot_type,bigversion,smallversion0,smallversion1
        else:
           return self.FAIL, -1, -1, -1, -1

    def get_hardware_error_status(self):
        ''' Get the error status of driver hardware
        '''
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x17) + struct.pack("B", 0x18)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           error_status, = struct.unpack('H', self.payload_args)
        #    print val0,val1,val2,val3
           return  self.SUCCESS, error_status
        else:
           return self.FAIL, -1

    def get_hardware_version(self):
        ''' Get the current version of the hardware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x13) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
           val0=0
           val1=1
           val2=0
           val3=1
           return  self.SUCCESS, val0, val1,val2,val3
        else:
           return self.FAIL, -1, -1, -1, -1

    def get_robot_status(self):
        ''' Get the status of automatic recharge.
        '''
        cmd_str=struct.pack("4B", self.HEADER0_SEND, self.HEADER1_SEND, 0x01, 0x19) + struct.pack("B", 0x1a)
        if (self.execute(cmd_str))==1 and self.check_sum_result == True:
            voltage_ob, recharge_ir_data_ob, status1_ob = struct.unpack('3H', self.payload_args) #zark
            return  self.SUCCESS, voltage_ob, recharge_ir_data_ob, status1_ob
        else:
            #print("getencoder  error!!!")
            return self.FAIL, -1, -1, -1


""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, ObRobot, ObRobot_top, base_frame):
        rospy.loginfo("......obrobot_driver_test_3_web_tool.py is running.....")

        self.ObRobot = ObRobot
        self.used_for_greetguest = rospy.get_param("~used_for_greetguest", False)

        if (self.used_for_greetguest is True) :
            self.ObRobot_top = ObRobot_top
        self.base_frame = base_frame
        # self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.rate = float(rospy.get_param("~rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
        self.useImu = rospy.get_param("~useImu", False)
        # self.useSonar = rospy.get_param("~useSonar", False)

        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.1518)
        self.wheel_track = rospy.get_param("~wheel_track", 0.375)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 42760)
        self.gear_reduction = rospy.get_param("~gear_reduction", 1.0)
        
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
                   
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * PI)
        
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.enc_left_last = None            # encoder readings
        self.enc_right_last = None        
        self.enc_left_speed = None            # encoder readings
        self.enc_right_speed = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.delta_time = 1         #delta time 20200609
        self.mcu_time_last = 1         #mcu time last 20200609
        self.mcu_time = 1         #mcu time 20200609
        self.cmd_x_last = 0             
        self.cmd_th_last = 0 
        self.x_mcu = 0                      # position in xy plane
        self.y_mcu = 0
        self.th_mcu = 0                     # rotation in radians

        self.yawangle = 0
        self.yawanglelast = 0
        self.resetencoderflag = False

        self.voltage = 0
        self.voltage_top = 0
        self.vxylast = 0
        self.vthlast = 0

        self.last_cmd_vel = now

        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

        self.lEncoderPub = rospy.Publisher('Lencoder', UInt16, queue_size=5)
        self.rEncoderPub = rospy.Publisher('Rencoder', UInt16, queue_size=5)
        self.lVelPub = rospy.Publisher('Lvel', Int16, queue_size=5)
        self.rVelPub = rospy.Publisher('Rvel', Int16, queue_size=5)
        self.yawAnglePub = rospy.Publisher('yaw_angle', Float32, queue_size=5)
        # self.whetherreachPub = rospy.Publisher('reach_goal', Int16, queue_size=5)
        #self.ObRobot.BEEP_SOUND()

        self.SUCCESS = 0
        self.FAIL = -1

        # self.voltage_val = 0
        self.voltage_pub = rospy.Publisher('voltage_value', Int32, queue_size=5, latch=True)
        self.voltage_top_pub = rospy.Publisher('voltage_top_value', Int32, queue_size=5, latch=True)
   
        self.emergencybt_val = 0
        self.emgy_stop_error_status = 0
        self.ob_emergencybt_pub = rospy.Publisher('ob_emergencybt_status', Int16, queue_size=5, latch=True)

        self.overcurrent_stop_error_status = 0#过流保护异常标志位

        #0: matching status
        #2: get the handshake signal
        #3: charging
        #4: charging and error 
        #5: battery is full and charge finish 
        self.recharge_status = 0
        self.recharge_status_top = 0
        self.recharge_status_last = -1
        self.recharge_status_top_last = -1


        self.error_status_driver = 0
        self.error_status_driver_last = -1        
        self.error_status_driver_top = 0
        self.error_status_driver_top_last = -1

        self.recharge_Index_control = 0
        self.rechargestatusperiod = 0.5 * self.rate #period is 0.1s self.rate=30
        self.voltage_value_period = 5 * self.rate #period is 0.1s self.rate=30
        self.get_error_status_period = 0.5 * self.rate#2s才能获取到错误状态
        # print("self.rechargestatusperiod", self.rechargestatusperiod)
        self.voltage_value_cnt = 0
        self.rechargestatuscnt = 0
        self.recharge_ir_data = 0
        self.recharge_v_left = 0
        self.recharge_v_right = 0

        self.get_error_status_cnt = 0

        self.unlock_lock_flag = False
        
    
        self.exit_recharge_flag = False
        self.exit_recharge_cmd = 0

        self.recharge_control_status = False

        self.location_error_status = False
        self.in_moving = 0
        self.hatch_bounced = False

        self.recharge_status_pub = rospy.Publisher('recharge_status', Int16, queue_size=5, latch=True)
        self.recharge_status_top_pub = rospy.Publisher('recharge_status_top', Int16, queue_size=5, latch=True)
        self.ir_data_pub = rospy.Publisher('ir_data', Int16, queue_size=5)
        self.error_status_driver_pub = rospy.Publisher('ob_error_status_driver', Int16, queue_size=5, latch=True)
        self.error_status_driver_top_pub = rospy.Publisher('ob_error_status_driver_top', Int16, queue_size=5, latch=True)
        self.locker_status_pub = rospy.Publisher('ob_locker_status', Int16, queue_size=5, latch=True)
        self.locker_status = 0
        self.locker_status_last = -1

        rospy.Subscriber('ob_locker_status_setting', Int16, self.ob_locker_status_setting_cb)

        rospy.Subscriber("encoder_reset", Int16, self.resetEncoderCallback)

        rospy.Subscriber('ob_unlock_locker', Int16, self.ob_unlock_locker_cb)

        rospy.Subscriber("driver_enable", Int16, self.enabledriverCallback)
        self.enable_driver_flag = True

        rospy.Subscriber("ob_exit_recharge", Byte, self.ob_exit_recharge_Callback)

        #订阅机器人充电状态话题：失败为false
        rospy.Subscriber("ob_recharge_control_status", Bool, self.ob_recharge_control_status_cb)
        rospy.Subscriber('ob_web_tool_runing', Bool, self.ob_web_tool_runing_cb)
        self.ob_web_tool_runing_Flag=True#在WEB部署工具driver文件中，默认标志位为True

        #订阅机器人定位状态话题：定位成功为true,定位失败为false
        rospy.Subscriber("ob_location_status", Bool, self.ob_location_status_cb)
        rospy.Subscriber("ob_relocation_finish", Bool, self.ob_relocation_finish_cb)
        self.relocationsucess_flag = False   #重定位是否成功
        self.relocationfinish_flag = False   #重定位完成标识符，只有完成才进入自动充电

        rospy.Subscriber('ob_in_moving', Int16, self.ob_in_moving_cb)

        #发送解除引起机器人卸力的异常,包括急停\过流\导航失败定位失败等
        rospy.Subscriber("ob_error_resolve_driver_control", Int16, self.ob_error_resolve_driver_control_cb)
        self.error_resolve_driver_control=0
        
        rospy.Subscriber("recharge_handle", Int16, self.recharge_handle_cb)
        self.in_recharing = 0

        # self.ObRobot_version=0
        _,car_robot_type,car_bigversion,car_smallversion0,car_smallversion1=self.ObRobot.get_firmware_version()
        self.slam_project_version = rospy.get_param("~slam_project_version",0)
        rospy.loginfo ("*************************************************")
        rospy.loginfo ("on-bright robot_type is "+str(car_robot_type))
        rospy.loginfo ("on-bright firmware_version is "+str(car_bigversion)+str(".")+str(car_smallversion0)+str(".")+str(car_smallversion1))
        rospy.loginfo ("on-bright slam version is "+str(self.slam_project_version))
        rospy.loginfo ("*************************************************")
        if (self.used_for_greetguest is True) :
            _,car_robot_type,car_bigversion,car_smallversion0,car_smallversion1=self.ObRobot_top.get_firmware_version()
            self.slam_project_version = rospy.get_param("~slam_project_version",0)
            rospy.loginfo ("*************************************************")
            rospy.loginfo ("on-bright robot_type is "+str(car_robot_type))
            rospy.loginfo ("on-bright top firmware_version is "+str(car_bigversion)+str(".")+str(car_smallversion0)+str(".")+str(car_smallversion1))
            rospy.loginfo ("*************************************************")
     


        # self.initialpose_listenning = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.ob_initialpose_recall)
    def recharge_handle_cb(self, data):
        self.in_recharing = data.data # 充电过程状态  0:not recharge 1:going to recharge point 2:matching the ir 3:start recharging


    def ob_error_resolve_driver_control_cb(self, data):
        self.error_resolve_driver_control = data.data



    def ob_web_tool_runing_cb(self, data):
        self.ob_web_tool_runing_Flag = data.data
    def ob_in_moving_cb(self, data):
        self.in_moving = data.data

    #监听是否有收到重定位话题，有则记录log
    # def ob_initialpose_recall(self, data):
    #     rospy.loginfo("/initialpose topic have been Subscriber")

    def ob_exit_recharge_Callback(self, data):
        if data.data == 1:
            self.exit_recharge_flag = True
            self.exit_recharge_cmd = 1
        elif data.data == 0:
            self.exit_recharge_flag = True
            self.exit_recharge_cmd = 0

    def ob_locker_status_setting_cb(self, data):
        self.locker_status = data.data



    def ob_unlock_locker_cb(self, data):
        if data.data == 1:
            print("receive unlock locker command")
            self.unlock_lock_flag = True

    def ob_recharge_control_status_cb(self, data):
        if data.data == False:
            if self.ob_web_tool_runing_Flag==False:
                self.recharge_control_status = True
            rospy.loginfo("get recharge_control_status: False")
        else:
            rospy.loginfo("get recharge_control_status: True")
            self.recharge_control_status = False

    def ob_relocation_finish_cb(self, data):
        if(data.data is True):
            self.relocationsucess_flag = True
        self.relocationfinish_flag = True

    def ob_location_status_cb(self, data):
        if data.data == False:
            if self.ob_web_tool_runing_Flag==False:
                self.location_error_status = True
            rospy.loginfo("get ob_location_status: False")
        else:
            rospy.loginfo("get ob_location_status: True")
            self.location_error_status = False
            self.hatch_bounced = False
            self.relocationsucess_flag = True   #重定位是否成功

    def enabledriverCallback(self, data):
        if data.data == 0:
            self.enable_driver_flag = False
        else:
            self.enable_driver_flag = True

    def resetEncoderCallback(self, req):
        if req.data==1:
            try:
                temp_send_try_cnt = 0
                while(self.ObRobot.reset_encoders()==self.FAIL and temp_send_try_cnt<5):
                    temp_send_try_cnt+=1
                    time.sleep(0.1)
                # if temp_send_try_cnt<5:
                #     self.resetencoderflag = True
                #     rospy.loginfo("reset encoder success")
                # else:
                #     rospy.loginfo("reset encoder fail")

            except:
                rospy.logerr("request to reset encoder exception ")
    
    def resetImuCallback(self, req):
        if req.data==1:
            try:
                res = self.ObRobot.reset_imu()
                if res==self.FAIL:
                    rospy.logerr("reset imu failed ")
            except:
                rospy.logerr("request to reset imu exception ")
        
    def poll(self):

        if True:
            # try:
            # 获取里程计
            if True:
                stat_,temp1,temp2,temp3,temp4,temp5,temp6 = self.ObRobot.get_encoder_counts()
                # print 'debug',stat_,temp1,temp2,temp3,temp4,temp5,temp6
                if stat_ == self.FAIL:
                    rospy.loginfo("ob car get encoder fail!!")
                    return 0
                else:
                    self.enc_left = temp1
                    self.enc_right = temp2
                    self.yawangle = temp3
                    self.enc_left_speed = temp4
                    self.enc_right_speed = temp5
                    self.mcu_time = temp6
                
                self.delta_time = self.mcu_time - self.mcu_time_last#mcu时间间隔
                if self.delta_time < 0:
                    self.delta_time += 65536

                self.delta_time /= 100000.0 #单位:s
                self.mcu_time_last = copy.deepcopy(self.mcu_time)

                # zark test 20200525 #
                now = rospy.Time.now()
                dt = now - self.then
                self.then = copy.deepcopy(now)
                dt = dt.to_sec()
                self.t_next = now + self.t_delta

                # print(dt, self.delta_time)

            # Calculate odometry 20200708 解算里程计
            if self.enc_left_last == None:
                dright_enc = 0
                dleft_enc = 0
            else:  

                dleft_enc = self.enc_left - self.enc_left_last
                # if(dleft_enc > 32767):
                #     dleft_enc = dleft_enc - 65536
                # elif(dleft_enc < -32767):
                #     dleft_enc = dleft_enc + 65536
                dright_enc = self.enc_right - self.enc_right_last
                # if(dright_enc > 32767):
                #     dright_enc = dright_enc - 65536
                # elif(dright_enc < -32767):
                #     dright_enc = dright_enc + 65536

            #判断左右轮的编码器脉冲值有没有发生跳变，有则记录log
            if abs(dleft_enc)>1000 or abs(dright_enc)>1000:
                # print("enc_left",self.enc_left, "enc_left_last",self.enc_left_last, "dleft_enc", dleft_enc)
                # print("enc_right",self.enc_right, "enc_right_last",self.enc_right_last, "dright_enc", dright_enc)
                # info = "encord error!!! "+"  enc_left: " + str(self.enc_left) + "  enc_left_last: " + str(self.enc_left_last) + "  dleft_enc: " + str(dleft_enc)
                # rospy.loginfo(info)
                # info = "encord error!!! "+"  enc_right: " + str(self.enc_right) + "  enc_right_last: " + str(self.enc_right_last) + "  dright_enc: " + str(dright_enc)
                # rospy.loginfo(info)

                if abs(dleft_enc)>1000:
                    dleft_enc=0
                if abs(dright_enc)>1000:
                    dright_enc=0                    

            self.enc_right_last = copy.deepcopy(self.enc_right)
            self.enc_left_last = copy.deepcopy(self.enc_left)
            #rospy.loginfo(self.enc_left,self.enc_left_last,dleft_enc)

            dleft = 1.0 * dleft_enc / self.ticks_per_meter 
            dright = 1.0 * dright_enc / self.ticks_per_meter 
            
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track

            if (dxy_ave != 0):
                self.x += (cos(self.th) * dxy_ave)
                self.y += (sin(self.th) * dxy_ave)
    
            self.th = self.yawangle / 180.0 * math.pi #zark 20200617
            
            # if (self.enc_left_speed > 32767):
            #     self.enc_left_speed -= 65536
            # if (self.enc_right_speed > 32767):
            #     self.enc_right_speed -= 65536

            vxy = (self.enc_left_speed - self.enc_right_speed) / 1000.0 / 2.0;  #线速度，单位:m/s               
            # vth = (-self.enc_left_speed - self.enc_right_speed) / 100.0 / self.wheel_track; 
            vth = (-self.enc_left_speed - self.enc_right_speed) / 1000.0 / self.wheel_track # * 1.08; #角速度，单位:rad/s

            # vth = vth * 0.1 + self.vthlast * 0.9 #debug
            # self.vthlast = vth #debug


    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
            
            #发布里程计,发布tf变换
            # Create the odometry transform frame broadcaster.
            if (self.useImu == False) :
                self.odomBroadcaster.sendTransform(
                  (self.x, self.y, 0), 
                  (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                  rospy.Time.now(),
                  self.base_frame,
                  "odom"
                )

            # print(self.x, self.y, self.yawangle, self.th/3.1415926*180)
    
            # 发布odom话题
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy 
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0
                self.cmd_x_last = 0             
                self.cmd_th_last = 0 
            
            self.v_left = self.v_des_left
            self.v_right = self.v_des_right

            #debug data
            # self.lVelPub.publish(self.v_left)
            # self.rVelPub.publish(self.v_right)      

            # zark
            if self.resetencoderflag is True:
                self.resetencoderflag = False
                self.enc_left = None            # encoder readings
                self.enc_right = None
                self.enc_left_last = None            # encoder readings
                self.enc_right_last = None                 
                self.x = 0                      # position in xy plane
                self.y = 0
                self.th = 0                     # rotation in radians
                self.v_left = 0
                self.v_right = 0
                self.v_des_left = 0             # cmd_vel setpoint
                self.v_des_right = 0
                self.cmd_x_last = 0             
                self.cmd_th_last = 0                
                # rospy.loginfo("reset encoder success ")

            # Set motor speeds in encoder ticks per PID loop
            if (not self.stopped):
                if (self.enable_driver_flag is True):
                    self.ObRobot.drive(int(self.v_left), int(self.v_right))
                            
            # get other driver status 获取机器人底层状态
            stat_, voltage_temp, recharge_ir_data_temp, status1_temp = self.ObRobot.get_robot_status()
            if stat_ == self.FAIL:
                rospy.loginfo("cannot get voltage and ir")
            else:
                #test code
                # self.locker_status = status1_temp & 0x1
                
                self.recharge_status = (status1_temp & 0xe)/2
                if(self.emergencybt_val != status1_temp & 0x10):
                    self.emergencybt_val = status1_temp & 0x10
                    self.ob_emergencybt_pub.publish(self.emergencybt_val >> 4)

                #如果急停按键被按下,则置为急停按键被按下标识符
                if self.emergencybt_val >> 4 == 1: 
                    self.emgy_stop_error_status = 1

                #如果急停按键被弹出且机器人处于充电状态,则清零急停按键被按下标识符
                if (self.in_recharing >= 3) and (self.emergencybt_val >> 4 == 0):
                    self.emgy_stop_error_status = 0

                #如果充电异常状态标志位被置位且机器人处于充电状态,则清零充电异常状态标志位
                if (self.in_recharing >= 3) and (self.recharge_control_status == True):
                    self.recharge_control_status = False
                    rospy.loginfo("clear self.recharge_control_status=False") 

                #如果定位异常状态标志位被置位且机器人处于充电状态,则清零定位异常状态标志位
                if (self.in_recharing >= 3) and (self.location_error_status == True):
                    self.location_error_status = False
                    self.relocationsucess_flag = True   #重定位是否成功
                    rospy.loginfo("clear self.location_error_status=False") 


                #收到手动异常解除命令,对异常进行解除判断
                if self.error_resolve_driver_control!=0:
                    rospy.loginfo("get error_resolve_driver_control!!!!!!")
                    self.error_resolve_driver_control=0
                    #收到手动清除异常操作,对各种异常状态进行判断并清除异常标志位和使能点击驻力
                  
                    #清除异常标志位   
                    if (self.emergencybt_val >> 4 == 0) and (self.emgy_stop_error_status == 1):
                        self.emgy_stop_error_status=0
                        rospy.loginfo("clear self.emgy_stop_error_status=0")
                    if self.location_error_status == True and self.relocationsucess_flag == True:
                        #如果不在充电桩上开机则无法通过手动方式恢复异常
                        self.location_error_status = False
                        rospy.loginfo("clear self.location_error_status=False") 
                    if self.recharge_control_status == True:
                        self.recharge_control_status = False
                        rospy.loginfo("clear self.recharge_control_status=False")
                    if self.hatch_bounced == True and self.locker_status == 0:
                        self.hatch_bounced = False
                        rospy.loginfo("clear self.hatch_bounced=False")

                    

                #如果急停按键被弹出则清零急停按键被按下标识符,并且使能电机驻力
                if(self.emergencybt_val >> 4 == 0) and (self.emgy_stop_error_status == 1):

                    temp_send_try_cnt = 0
                    while(self.ObRobot.exit_recharge(Byte(1))==self.FAIL and temp_send_try_cnt<5):
                        temp_send_try_cnt+=1
                        time.sleep(0.1)

                    time.sleep(0.1)
                    temp_send_try_cnt = 0
                    while(self.ObRobot.exit_recharge(Byte(0))==self.FAIL and temp_send_try_cnt<5):
                        temp_send_try_cnt+=1
                        time.sleep(0.1)
                        
                    if self.ob_web_tool_runing_Flag==True:
                        self.emgy_stop_error_status = 0

                if(self.unlock_lock_flag is True):
                    self.unlock_lock_flag = False
                    temp_send_try_cnt = 0
                    while(self.ObRobot.set_unlock_locker()==self.FAIL and temp_send_try_cnt<5):
                        temp_send_try_cnt+=1
                        time.sleep(0.1)
                    # if temp_send_try_cnt<5:
                    #     rospy.loginfo("unlock locker success")
                    # else:
                    #     rospy.loginfo("unlock locker fail")                       

                if(self.exit_recharge_flag is True):
                    self.exit_recharge_flag = False
                    temp_send_try_cnt = 0
                    while(self.ObRobot.exit_recharge(Byte(self.exit_recharge_cmd))==self.FAIL and temp_send_try_cnt<5):
                        temp_send_try_cnt+=1
                        time.sleep(0.1)
                    # if temp_send_try_cnt<5:
                    #     rospy.loginfo("Motor force success")
                    # else:
                    #     rospy.loginfo("Motor force fail") 


                self.voltage = voltage_temp
                self.recharge_ir_data = recharge_ir_data_temp

            if (self.used_for_greetguest is True) :
                # get other driver status(top)获取上层板的状态
                stat_, voltage_temp, recharge_ir_data_temp, status1_temp = self.ObRobot_top.get_robot_status()
                if stat_ == self.FAIL:
                    rospy.loginfo("cannot get voltage and ir")
                else:
                    self.recharge_status_top = (status1_temp & 0xe)/2
                    self.voltage_top = voltage_temp
                    # print "self.voltage_top", self.voltage_top

            self.get_error_status_cnt += 1
            if(self.get_error_status_cnt > self.get_error_status_period):
                self.get_error_status_cnt = 0
                stat_, error_status_driver, = self.ObRobot.get_hardware_error_status()
                if stat_ == self.FAIL:
                    rospy.loginfo("get_hardware_error_status fail")
                else:
                    #0x02或0x04是表示发生电机过流保护
                    if self.location_error_status == True:
                        error_status_driver = error_status_driver | 0x08    #导航失败,定位丢失
                    if self.recharge_control_status == True:
                        error_status_driver = error_status_driver | 0x40    #充电异常

                    if self.emgy_stop_error_status == 1:
                        error_status_driver = error_status_driver | 0x01    #急停按键被按下

                    if (error_status_driver&0x02) or (error_status_driver&0x04):
                        self.overcurrent_stop_error_status = 1#过流保护异常标志位
                    else:
                        self.overcurrent_stop_error_status = 0#过流保护异常标志位

                    if (self.in_moving != 0) and (self.locker_status == 1) and (self.hatch_bounced is False):
                        rospy.loginfo("The hatch bounced open during movement,disable the motor driver")
                        self.in_moving = 0
                        self.hatch_bounced = True

                    if self.hatch_bounced is True:
                        error_status_driver = error_status_driver | 0x10 #仓门异常
                    
                    #更改话题发送为有值变化才发送
                    if self.locker_status!=self.locker_status_last:
                        self.locker_status_pub.publish(self.locker_status)
                    self.locker_status_last = self.locker_status

                    self.error_status_driver=error_status_driver
                    self.error_status_driver_pub.publish(self.error_status_driver)
                    # if self.error_status_driver!=self.error_status_driver_last:
                    #     self.error_status_driver_pub.publish(self.error_status_driver)
                    self.error_status_driver_last = self.error_status_driver

                if (self.used_for_greetguest is True) :
                    stat_, error_status_driver, = self.ObRobot_top.get_hardware_error_status()
                    if stat_ == self.FAIL:
                        rospy.loginfo("get_hardware_error_status top fail")
                    else:
                        self.error_status_driver_top=error_status_driver
                        if self.error_status_driver_top!=self.error_status_driver_top_last:
                            self.error_status_driver_top_pub.publish(self.error_status_driver_top)
                        self.error_status_driver_top_last = self.error_status_driver_top
            
            # debug data
            # self.lEncoderPub.publish(self.enc_left)
            # self.rEncoderPub.publish(self.enc_right)
            # self.yawAnglePub.publish(self.yawangle)
            self.ir_data_pub.publish(self.recharge_ir_data)

            # 发布电压信息，由于安卓平板要求话题不要发太快
            self.voltage_value_cnt += 1
            if self.voltage_value_cnt >= self.voltage_value_period:
                self.voltage_value_cnt = 0
                self.voltage_pub.publish(self.voltage)
                if (self.used_for_greetguest is True) :
                    self.voltage_top_pub.publish(self.voltage_top)
            
            #发布充电状态信息
            self.rechargestatuscnt += 1
            if self.rechargestatuscnt >= self.rechargestatusperiod:
                self.rechargestatuscnt = 0

                #更改话题发送为有值变化才发送
                if self.recharge_status!=self.recharge_status_last:
                    self.recharge_status_pub.publish(self.recharge_status)
                self.recharge_status_last = self.recharge_status
                if (self.used_for_greetguest is True) :
                    if self.recharge_status_top!=self.recharge_status_top_last:
                        self.recharge_status_top_pub.publish(self.recharge_status_top)
                    self.recharge_status_top_last = self.recharge_status_top                

    def stop(self):
        self.stopped = True
        # self.ObRobot.drive(0, 0)
        temp_send_try_cnt = 0
        while(self.ObRobot.drive(0, 0)==self.FAIL and temp_send_try_cnt<5):
            temp_send_try_cnt+=1
            time.sleep(0.1)
        # if temp_send_try_cnt<5:
        #     rospy.loginfo("drive success")
        # else:
        #     rospy.loginfo("drive fail")  

    def isPassedCallback(self, msg): 
        if(msg.data>2):
            self.isPassed = False
        else:
            self.isPassed = True

    def isPassedCallback_2(self, msg):
        if(msg.data>2):
            self.isPassed_2 = False
        else:
            self.isPassed_2 = True

    # 收到目标速度后的回调        
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        robot_cmd_vel = Twist()
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        #增加防止启动速度过快导致机器人翘头
        # if abs(self.cmd_x_last-x)>0.1:
        #     x=self.cmd_x_last+(x-self.cmd_x_last)*0.1
        # if abs(self.cmd_th_last-th)>0.1:
        #     th=self.cmd_th_last+(th-self.cmd_th_last)*0.1
        # self.cmd_x_last = x             
        # self.cmd_th_last = th

        left = x - th * self.wheel_track  * self.gear_reduction / 2.0
        right = x + th * self.wheel_track  * self.gear_reduction / 2.0

        self.v_des_left = (left * 1000.0) #单位转换为mm/s下发
        self.v_des_right = (right * 1000.0)  #单位转换为mm/s下发         
        

class ObRobotROS():
    def __init__(self):
        rospy.init_node('ObRobot', log_level=rospy.DEBUG)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        # top 20200601
        self.port_top = rospy.get_param("~port2", "/dev/ttyUSB1")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        
        self.use_base_controller = rospy.get_param("~use_base_controller", True)
        
        self.used_for_greetguest = rospy.get_param("~used_for_greetguest", False)
        
        # Initialize the controlller
        self.controller = ObRobot(self.port, self.baud, self.timeout)
        if self.used_for_greetguest is True:
            # top 20200601
            self.controller_top = ObRobot(self.port_top, self.baud, self.timeout)
        
        # Make the connection
        self.controller.connect()
        if self.used_for_greetguest is True:
            # top 20200601
            self.controller_top.connect()
        
        rospy.loginfo("Connected to OB robot car on port " + self.controller.port.name + " at " + str(self.baud) + " baud")
        if self.used_for_greetguest is True:
            # top 20200601
            rospy.loginfo("Connected to OB robot car top on port " + self.controller_top.port.name + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = thread.allocate_lock()
              
        # Initialize the base controller if used
        if self.use_base_controller:
            if self.used_for_greetguest is True:
                self.myBaseController = BaseController(self.controller, self.controller_top, self.base_frame)
            else:
                self.myBaseController = BaseController(self.controller, None, self.base_frame)
    
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
                    
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                #rospy.loginfo("poll test")
                mutex.release()
            r.sleep()
    
    def shutdown(self):
        # Stop the robot
        try:
            # rospy.loginfo("Stopping the robot...")
            temp_send_try_cnt = 0
            while(self.controller.drive(0,0)==self.FAIL and temp_send_try_cnt<5):
                temp_send_try_cnt+=1
                time.sleep(0.1)
            # if temp_send_try_cnt<5:
            #     rospy.loginfo("drive success")
            # else:
            #     rospy.loginfo("drive fail")
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down ObRobot Node...")
        
if __name__ == '__main__':
    myObRobot = ObRobotROS()
