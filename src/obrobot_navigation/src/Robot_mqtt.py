#!/usr/bin python
#encoding=utf-8

#author: francis
#date: 2021.03.04
#version:210525
#description: all function tegether:phonecall,simulate elevator,order,

from __future__ import print_function
import paho.mqtt.client as mqtt
from datetime import datetime
import socket
import json
import time
import threading
import urllib2

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32,Int16
from geometry_msgs.msg import Twist
class Robot_mqtt():
    def __init__(self, robot_id):

        ##### mqtt #####
        # self.address = '47.110.228.31'
        self.address = '47.96.100.153'
        self.port = 1883
        self.client_id = robot_id
        self.user = 'litu'
        self.pwd = 'litu@hotel2021'

        self.client = mqtt.Client(self.client_id)
        self.client.username_pw_set(self.user, self.pwd)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        # self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = self.on_disconnect
        # self.client.on_publish = self.on_publish
        ################

        self.robot_for_education = True#True #科教版机器人功能标识
        self.robot_for_meal_delivery = False#False

        self.connectionState = False
        self.disconnect_status = False
        self.pubMessageState = False #True: success False:failed

        ##### elevator #####
        self.getElevatorState_success = True
        self.Header = bytearray([0x49, 0x54, 0x4C])
        self.openstate=False
        self.subID=0
        self.floor=0
        self.frontdoor=''
        self.emergency='0'
        self.mode=0
        self.eleSubTopic = '/robot/'+ self.client_id +'/update'
        self.elePubTopic = '/robot/'+ self.client_id +'/get'
        self.receive_cb_cnt = 0
        self.reach_floor = True
        self.reach_floor_cnt = 0
        self.electrlPubResult = False
        self.readElePubResult = False
        ####################

        ##### order report #####
        self.orderId = 0
        self.room = 0
        self.startTime = 0
        self.endTime = 0
        self.pickTime = 0
        self.orderPubTopic = '/robot/'+ self.client_id +'/delivery/report'
        self.orderResultSubTopic = '/robot/'+ self.client_id +'/delivery/report/response'
        self.orderSendResult = False
        ########################

        ##### htrip order #####
        self.callId = ''
        self.roomNo = ''
        self.orderSN = ''
        self.fetchNo = ''
        self.accessCode = ''
        self.catelogName = ''
        self.remarks = ''
        self.createTime = ''

        self.tempcallId = ''
        self.temproomNo = ''
        self.temporderSN = ''
        self.tempfetchNo = ''
        self.tempaccessCode = ''
        self.tempcatelogName = ''
        self.tempremarks = ''
        self.tempcreateTime = ''

        self.orderReceiveTopic = '/robot/' + self.client_id + '/order'
        self.orderStatusPubTopic = '/robot/' + self.client_id + '/order/reportStatus'

        ## order status ##
        # 1:
        # 2:
        # 3:
        # 4:
        # 5:
        self.orderStatus = 0
        self.roomIdFromhead = 0
        

        # rospy.init_node('order_node', anonymous=True)
        self.orderROSinfoPub = rospy.Publisher('ob_delivery_order_info_id', Twist, queue_size=5, latch=True)
        self.orderROSgoodsPub = rospy.Publisher('ob_delivery_order_info_goods', String, queue_size=5, latch=True)
        self.robotstatusPub = rospy.Publisher('ob_robot_status', Int32, queue_size=5)
        rospy.Subscriber('ob_delivery_order_status', Twist, self.order_status_cb)
        self.orderMsg = Twist()
        self.orderGoods = String()
        ########################

        ##### text phone #####
        self.textPubTopic = '/robot/' + self.client_id +'/sms'
        ######################

        ##### check robot state #####
        self.ele_occ_status = 0
        self.floor_occ_status = 0
        self.avoidpoint_occ_status = 0
        self.robot_working_status = 0



        #robot elevator state
        self.notInEle = 0
        self.waitingForEle = 1
        self.comingEle = 2
        self.takingEle = 3
        self.getOutEle = 4

        #self state
        self.thisStatus = 0 #self status
        self.thisWorking = 0
        self.thisFloor = ''
        self.thisAvoidpoint = ''
        self.thisBattery = ''
        self.thisWarehouse = ''
        self.thisX = 0
        self.thisY = 0


        self.ob_error_status=0
        self.robotCnt = 0
        self.getRobotState_success = True
        self.floorOccupiedSet = set() # empty set
        self.eleOccupiedState = False
        self.avoidOccupiedSet = set()
        self.statusSet = set()
        self.robotList = []
        self.statePubTopic = '/robot/' + self.client_id + '/status/report'
        self.checkRobotPubTopic = '/robot/' + self.client_id + '/status/query'
        self.checkRobotSubTopic = '/robot/' + self.client_id + '/status/response'
        self.robotPubResultSubTopic = '/robot/' + self.client_id + '/status/report/response'

        self.debuginfoPubTopic = '/robot/' + self.client_id + '/pub/log'

        self.robotPubResult = False
        #############################

        ### phone call ###
        self.phonecallStatus = 0
        self.phoneCallTopic = '/robot/' + self.client_id + '/call'
        self.phoneSubTopic = '/robot/' + self.client_id + '/call/response'
        # self.phoneAnswertoHeadROS = rospy.Publisher('ob_callback_phone_doorbell_id', Int16, queue_size=1)
        rospy.Subscriber('ob_start_phone_doorbell_id', Int32, self.phone_call_cb)
        ##################

        self.msg_floor = "0"
        self.msg_room = "0"
        self.msg_location = "0"
        self.msg_status = "0"

        self.checkNetwork_pass = False
        #定义mqtt数据通信状态标志位，用于转到ROS节点进行mqtt的心跳判断程序对网络连接状态进行话题发布
        #0：空闲
        #1：发布mqtt数据通信异常状态
        #2：发布mqtt数据通信正常状态
        self.flag_mqtt_comm_fail_pub=0
        self.TH_mqtt_request_cnt = 20
        self.TH_cmd_repeat_timer = 0.5

        self.nav_step=0

    def checkNetwork(self):
        #import urllib2
        url = 'http://www.baidu.com'
        timeout = 2
        try:
            urllib2.urlopen(url, timeout=timeout)

        except urllib2.URLError as err:
            print('Network not found')
            self.checkNetwork_pass = False
        else:
            print('Network connected')
            self.checkNetwork_pass = True
            

    # htrip order state
    def order_status_cb(self, data):
        orderstate = str(int(data.linear.x))
        if orderstate != '6':# 6 is busy
            self.orderStatus = orderstate
            self.roomIdFromhead = str(int(data.linear.y))
            self.roomNo = self.roomIdFromhead

            # update temp status
            self.callId = self.tempcallId
            # self.roomNo = self.temproomNo
            self.orderSN = self.temporderSN
            self.fetchNo = self.tempfetchNo
            self.accessCode = self.tempaccessCode
            self.catelogName = self.tempcatelogName
            self.remarks = self.tempremarks
            self.createTime = self.tempcreateTime

            # update order status to server
            self.send_orderStatus(self.roomIdFromhead, self.orderSN, self.orderId, self.orderStatus)

        elif orderstate == '6':
            self.tempcallId = self.callId
            self.temproomNo = self.roomNo
            self.temporderSN = self.orderSN
            self.tempfetchNo = self.fetchNo
            self.tempaccessCode = self.accessCode
            self.tempcatelogName = self.catelogName
            self.tempremarks = self.remarks
            self.tempcreateTime = self.createTime

    def phoneCall(self, roomNo):
        if self.robot_for_education == False: #科教版机器人功能标识
 
        
            msg = {
                "roomNo":str(roomNo)
            }
            js_msg = json.dumps(msg)
            if self.connectionState is True:
                try:
                    self.client.publish(self.phoneCallTopic, payload=js_msg, qos=1)
                    print('Call Room:', roomNo)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish Call Room Error')
                else:
                    self.pubMessageState = True 

    def phone_call_cb(self, data):
        room = data.data
        if self.nav_step==15:
            rospy.loginfo("Publish Phone Call Room"+str(room))
            self.phoneCall(room)
        else:
            rospy.loginfo("Publish Phone Call Room"+str(room)+"Error, not in reach room status")

    def getXor(self, array):
        result = 0x00
        for i in array:
            result = result ^ i
        return result

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connectionState = True
            self.disconnect_status = False
            self.client.subscribe(self.eleSubTopic)
            self.client.subscribe(self.checkRobotSubTopic)
            self.client.subscribe(self.orderReceiveTopic)
            self.client.subscribe(self.phoneSubTopic)
            self.client.subscribe(self.robotPubResultSubTopic)
            self.client.subscribe(self.orderResultSubTopic)
            rospy.loginfo('on_connect:Connect to MQTT successfully.')
        else:
            self.connectionState = False
            rospy.loginfo('on_connect:Failed to connect MQTT')

    # def on_publish(self, client, userdata, mid):
        # print("On publish success")

    def on_message(self, client, userdata, msg):
        # print('In On Message')
        # answer = str(msg.payload.decode('utf-8'))
        answer = str(msg.payload)
        # print('Answer:', answer)
        # print('Answer type:', type(answer))
        if msg.topic == self.eleSubTopic:
            self.receive_cb_cnt = 0
            self.subID = "0"
            self.floor = "0"
            self.frontdoor = "0"
            count = 0 # packet count
            jsList = []
            snrList = []
            for index, ch in enumerate(answer):
                if ch == 'I':
                    if answer[index+1] == 'T' and answer[index+2] == 'L':
                        count = count + 1
                        ten = ord(answer[index+3])
                        packLen = ord(answer[index+4])
                        length = 256*ten + packLen
                        #res = answer[index+9:index+length+3] #real elevator
                        res = answer[index+9:index+length+4] #simulator

                        jsList.append(res)
                        snrList.append(answer[index+6])

            for i in range(count):
                if jsList[i] != '':
                    # print('i:', i)
                    # check SNR
                    snr = snrList[i] #get SNR byte
                    js_ans = json.loads(jsList[i].replace(' ', ''))
                    if snr == '\x01':
                        print('From openReportState recv')
                    elif snr == '\x03':
                        # print('From elevatorCtrl recv')
                        if js_ans["msg"] == "success" and js_ans["msgCode"] == 0:
                            # print('Elevator Control Command Send successfully')
                            # self.comand_send_bool = True
                            self.electrlPubResult = True
                        else:
                            self.electrlPubResult = False
                    elif snr == '\x04':
                        # print('From readElevatorState recv')
                        if js_ans["msg"] == "success" and js_ans["msgCode"] == 0:
                            # print('Read Elevator Send successfully')
                            # self.comand_send_bool = True
                            self.readElePubResult = True
                        else:
                            # self.comand_send_bool = False
                            self.readElePubResult = False
                        if js_ans.has_key("status"):
                            #receive status msg
                            self.getElevatorState_success = True
                            if js_ans["status"][0]["floor"] != '-':
                                self.subID = js_ans["status"][0]["subID"]
                                self.floor = js_ans["status"][0]["floor"]
                                self.frontdoor = js_ans["status"][0]["frontDoor"]
                                if not "emergency" in js_ans["status"][0]:
                                    # print('no emergency key')
                                    pass
                                else:
                                    self.emergency = js_ans["status"][0]["emergency"]
                            else:
                                self.subID = js_ans["status"][0]["subID"]
                                self.floor = "-1"
                                self.frontdoor = js_ans["status"][0]["frontDoor"]
                                if not "emergency" in js_ans["status"][0]:
                                    # print('no emergency key')
                                    pass
                                else:
                                    self.emergency = js_ans["status"][0]["emergency"] 
                            # print('break in :'+str(i))
                            break 

                    elif snr == '\x05':
                        print('From setElevatorMode recv')
                    elif snr == '\x06':
                        print('From readElevatorMode recv')
                    else:
                        print('Error')
                    # print('--------------------------')
        elif msg.topic == self.checkRobotSubTopic:
            # clear
            robotCnt = 0 #记录除了自己外的当前酒店环境机器人数目
            robotList = []
            floorOccupiedSet = set()#记录当前酒店环境下所有机器人楼层占用信息
            statusSet = set()#记录当前酒店环境下所有机器人电梯占用信息
            avoidOccupiedSet = set()#记录当前酒店环境下所有机器人楼层避让点占用信息

            # print('Recive message: ', self.getRobotState_success)
            #process data
            begin = answer.find('[')
            end = answer.find(']')
            if begin == -1 or end == -1:
                print('Error: Message format')
            else:
                data = json.loads(answer[begin:end+1]) #only intercept first message
                # print('Data: \n',data)
                for index, value in enumerate(data):
                    # if data[index]['serialId']!=self.client_id and data[index]['hotelId']==11:
                    if data[index]['serialId']!=self.client_id:
                        robotCnt += 1
                        # robotList.append(data[index])
                        # if data[index].has_key('working'):
                        #     self.thisWorking = 
                        if data[index].has_key('floor'):
                            floorOccupiedSet.add(data[index]['floor'])
                            robotList.append(data[index])
                        if data[index].has_key('status'):
                            statusSet.add(data[index]['status'])
                        if data[index].has_key('avoidpoint'):
                            avoidOccupiedSet.add(data[index]['avoidpoint'])
                        self.getRobotState_success = True
                        # print('Recive message: ', self.getRobotState_success)
                    ### for one robot ###
                    elif data[index]['serialId'] == self.client_id:
                        self.getRobotState_success = True
                    #####################
                self.robotCnt = robotCnt
                self.robotList = robotList
                self.floorOccupiedSet = floorOccupiedSet
                self.statusSet = statusSet
                self.avoidOccupiedSet = avoidOccupiedSet
        elif msg.topic == self.robotPubResultSubTopic:
            #receive result from server
            data = json.loads(answer)
            if data["result"] == 0:
                self.robotPubResult = False
            else:
                self.robotPubResult = True
            # print("Robot State Pub result:", data["result"])

        elif msg.topic == self.orderResultSubTopic:
            data = json.loads(answer)
            if data["result"] == 1:
                self.orderSendResult = True
            else:
                self.orderSendResult = False
            # print('Order Send result:', data["result"])

        elif msg.topic == self.orderReceiveTopic:
            # print('In order receive topic')
            data = json.loads(answer)
            # print('data: ',data)
            # print('type of data:', type(data))
            # data = answer
            if data.has_key('roomNo'):
                self.temproomNo = data['roomNo']
            if data.has_key('orderSN'):
                self.temporderSN = data['orderSN']
            if data.has_key('fetchNo'):
                self.tempfetchNo = data['fetchNo']
            if data.has_key('accessCode'):
                self.tempaccessCode = data['accessCode']
            if data.has_key('catelogName'):
                # print('catelog:', data['catelogName'])
                self.tempcatelogName = data['catelogName']
                self.tempcatelogName = self.tempcatelogName.replace(',','\n')
            if data.has_key('remarks'):
                self.tempremarks = data['remarks']
            if data.has_key('createTime'):
                self.tempcreateTime = data['createTime']
            # self.showOrder()
            self.orderMsg.linear.x = int(self.temproomNo)
            self.orderMsg.linear.y = int(self.tempfetchNo)
            self.orderMsg.linear.z = int(self.tempaccessCode)
            self.orderGoods.data = self.tempcatelogName
            
            self.orderROSgoodsPub.publish(self.orderGoods)
            rospy.sleep(2)
            self.orderROSinfoPub.publish(self.orderMsg)
        elif msg.topic == self.phoneSubTopic:
            # print('In phone call topic')
            js_msg = json.loads(answer)
            self.phonecallStatus = js_msg["status"]
            # self.phoneAnswertoHeadROS.publish(self.phonecallStatus)
        # #
        # self.sendRobotState_success = True  

    # def on_subscribe(self, client, userdata, mid, granted_qos):
    #     print("On Subscribed: qos = %d" % granted_qos)

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print('Unexpected disconnection %s' % rc)
            self.disconnect_status = True
            self.connectionState = False
            self.openstate = False

    def connectToServer(self):
        self.client.connect(self.address, keepalive=15)
        self.client.loop_start()

    def openReportState(self,timeout=1):
        if self.robot_for_education == False: #科教版机器人功能标识
   
            """
            Open elevator report state. if timeout not zero, return elevator state 

            Parameters
            ----------
            timeout: 0-999
            0: close report state
            1-998: after 1-998 sec, elevator stop reporting state
            999: report state for no timeout
            """
            DIR = bytearray([0x00])
            SNR = bytearray([0x01])
            CMD = bytearray([0x10, 0x03])
            info_list = list(json.dumps({"fOpenCtrl":str(timeout)}))
            for i in range(info_list.count(' ')):
                info_list.remove(' ')
            INFO = bytearray(info_list)
            # print(self.INFO)
            LEN = bytearray([0x00, len(DIR) + len(SNR) + len(CMD) + len(INFO) + 1]) #1 for chk byte
            CHK = bytearray([self.getXor(LEN) ^ self.getXor(DIR) ^ self.getXor(SNR) ^ self.getXor(CMD) ^ self.getXor(INFO)])
            request = self.Header + LEN + DIR + SNR + CMD + INFO + CHK
            # for j in request:
            #     print('%x' %j,end=' ')
            # print(' ')

            if self.connectionState is True:
                try:
                    self.client.publish(self.elePubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish Open State Error')
                else:
                    self.pubMessageState = True

            self.openstate = True

    def readElevatorState(self):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            DIR = bytearray([0x00])
            SNR = bytearray([0x04])
            CMD = bytearray([0x10, 0x04])
            LEN = bytearray([0x00, len(DIR) + len(SNR) + len(CMD) + 1]) #1 for chk byte
            CHK = bytearray([self.getXor(LEN) ^ self.getXor(DIR) ^ self.getXor(SNR) ^ self.getXor(CMD)])
            request = self.Header + LEN + DIR + SNR + CMD + CHK
            
            #对获取电梯状态函数进行了优化，增加了是否获取成功的判断和重发机制
            self.getElevatorState_success = False
            if self.connectionState is True:
                try:
                    self.client.publish(self.elePubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish read Elevator State Error')
                else:
                    self.pubMessageState = True
            rospy.sleep(self.TH_cmd_repeat_timer)
            cnt_readElevatorState = 0
            while(self.getElevatorState_success is False):
                # rospy.loginfo("waiting for getElevatorState_success")
                cnt_readElevatorState+=1
                if cnt_readElevatorState>=self.TH_mqtt_request_cnt:
                    self.flag_mqtt_comm_fail_pub=1#1：发布mqtt数据通信异常状态
                    cnt_readElevatorState=self.TH_mqtt_request_cnt

                if self.connectionState is True:
                    try:
                        self.client.publish(self.elePubTopic, payload=request, qos=1)
                    except socket.error:
                        self.pubMessageState = False
                        rospy.loginfo('Publish read Elevator State Error')
                    else:
                        self.pubMessageState = True
                rospy.sleep(self.TH_cmd_repeat_timer)            
                if self.getElevatorState_success is True or self.ob_error_status!=0:
                    if cnt_readElevatorState==self.TH_mqtt_request_cnt:
                        self.flag_mqtt_comm_fail_pub=2#2：发布mqtt数据通信正常状态
                    break
            # if self.getElevatorState_success is True:
            #     rospy.loginfo("getElevatorState succeed")



    def elevatorCtrl(self, fOpenCtrl='', fCloseCtrl='', inCall='', startingFloor='', gotoFloor='', upDown=''):
        if self.robot_for_education == False: #科教版机器人功能标识
 
        
            """
            Control Elevator

            Parameters
            ----------
            fOpenCtrl: 0-99
                0: unpress open door button
                1: press open door button once
                2-99: press open door button for 2-99 sec
            
            fCloseCtrl: 0-99
                0: unpress close door button
                1: press close door button shortly
                2-99: press close door button for 2-99 sec
            
            inCall: Device(elevator) will light up floor and go there

            upDown: 
                1: down
                2: up
            """
            DIR = bytearray([0x00])
            SNR = bytearray([0x03])
            CMD = bytearray([0x10, 0x03])
        
            command_state = True

            if fOpenCtrl != '':
                fopenlist = list(json.dumps({"fOpenCtrl":fOpenCtrl}))
            else:
                fopenlist = []

            if fCloseCtrl != '':
                fcloselist = list(json.dumps({"fCloseCtrl":fCloseCtrl}))
            else:
                fcloselist = []

            if inCall != '':
                incalllist = list(json.dumps({"inCall":inCall}))
            else:
                incalllist = []

            # In this version cmdver should be 2
            cmdVer = 2

            # call = {"outCall":
            #             {"cmdVer":"2","startingFloor":"5","gotoFloor":"8","upDown":"2"}
            #        }

            if startingFloor != '' and gotoFloor != '':
                outcall = {"outCall":
                            {"cmdVer":str(cmdVer),"startingFloor":str(startingFloor),"gotoFloor":str(gotoFloor),"upDown":str(upDown)}
                        }
                outcalllist = list(json.dumps(outcall))
            else:
                outcalllist = []

            info_list = fopenlist + fcloselist + incalllist + outcalllist
            for i in range(info_list.count(' ')): #remove ' ' in json data
                info_list.remove(' ')
            INFO = bytearray(info_list)

            LEN = bytearray([0x00, len(DIR) + len(SNR) + len(CMD) + len(INFO) + 1])
            CHK = bytearray([self.getXor(LEN) ^ self.getXor(DIR) ^ self.getXor(SNR) ^ self.getXor(CMD) ^ self.getXor(INFO)])
            request = self.Header + LEN + DIR + SNR + CMD + INFO + CHK

            self.electrlPubResult = False
            if self.connectionState is True:
                try:
                    self.client.publish(self.elePubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish Elevator Ctrl Error')
                else:
                    self.pubMessageState = True
            rospy.sleep(self.TH_cmd_repeat_timer)
            cnt_electrlState = 0
            while(self.electrlPubResult is False):
                # rospy.loginfo("waiting for electrlPubResult")
                cnt_electrlState+=1
                if cnt_electrlState>=self.TH_mqtt_request_cnt:
                    self.flag_mqtt_comm_fail_pub=1#1：发布mqtt数据通信异常状态
                    cnt_electrlState=self.TH_mqtt_request_cnt

                if self.connectionState is True:
                    try:
                        self.client.publish(self.elePubTopic, payload=request, qos=1)
                    except socket.error:
                        self.pubMessageState = False
                        rospy.loginfo('Publish Elevator Ctrl Error')
                    else:
                        self.pubMessageState = True
                rospy.sleep(self.TH_cmd_repeat_timer)            
                if self.electrlPubResult is True or self.ob_error_status!=0:
                    if cnt_electrlState==self.TH_mqtt_request_cnt:
                        self.flag_mqtt_comm_fail_pub=2#2：发布mqtt数据通信正常状态
                    break
            # if self.electrlPubResult is True:
            #     rospy.loginfo("Elevator Ctrl publish succeed")


    def setElevatorMode(self, mode):
        if self.robot_for_education == False: #科教版机器人功能标识
   
            DIR = bytearray([0x00])
            SNR = bytearray([0x05])
            CMD = bytearray([0x10, 0x05])
            info_list = list(json.dumps({"mode":str(mode)}))
            for i in range(info_list.count(' ')):
                info_list.remove(' ')
            INFO = bytearray(info_list)
            LEN = bytearray([0x00, len(DIR) + len(SNR) + len(CMD) + len(INFO) + 1]) #1 for chk byte
            CHK = bytearray([self.getXor(LEN) ^ self.getXor(DIR) ^ self.getXor(SNR) ^ self.getXor(CMD) ^ self.getXor(INFO)])

            request = self.Header + LEN + DIR + SNR + CMD + INFO + CHK

            if self.connectionState is True:
                try:
                    self.client.publish(self.elePubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish eleCtrl Error')
                else:
                    self.pubMessageState = True

    def readElevatorMode(self):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            DIR = bytearray([0x00])
            SNR = bytearray([0x06])
            CMD = bytearray([0x10, 0x06])
            LEN = bytearray([0x00, len(DIR) + len(SNR) + len(CMD) + 1]) #1 for chk byte
            CHK = bytearray([self.getXor(LEN) ^ self.getXor(DIR) ^ self.getXor(SNR) ^ self.getXor(CMD)])
            request = self.Header + LEN + DIR + SNR + CMD + CHK

            if self.connectionState is True:
                try:
                    self.client.publish(self.elePubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish eleCtrl Error')
                else:
                    self.pubMessageState = True

    def closeConnectToServer(self):
        self.client.disconnect()
        self.connectionState = False
        self.openstate = False
        # print("********close Connect To Server********")

    def send_message(self, phone, floor, room, location, status):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            request = {
                "phone": str(phone),
                "floor": str(floor),
                "room": str(room),
                "location": str(location),
                "status": str(status)
            }
            #test code
            request = str(request)
            if self.connectionState is True:
                try:
                    self.client.publish(self.textPubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish message Error')
                else:
                    self.pubMessageState = True

    def send_orderStatus(self, roomNo, orderSN, orderId, status):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            msg = {
                "roomNo":roomNo,
                "orderSN":orderSN,
                "orderId":orderId,
                "status":status
            }
            # request = str(msg)
            request = json.dumps(msg)
            if self.connectionState is True:
                try:
                    self.client.publish(self.orderStatusPubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    # rospy.loginfo('Publish order status Error')
                else:
                    self.pubMessageState = True
                    # print('published order status')

    ## debug ##
    def showOrder(self):
        order = {
            "roomNo":self.roomNo,
            "orderSN":self.orderSN,
            "fetchNo":self.fetchNo,
            "accessCode":self.accessCode,
            "catelogName":self.catelogName,
            "remarks":self.remarks,
            "createTime":self.createTime
        }
        # js = json.dumps(order, sort_keys=True, indent=4, separators=(',', ':'))
        # print(order)
       

    def startDeliverReport(self, room, startTime):
        if self.robot_for_education == False: #科教版机器人功能标识
 
        
            self.orderId = self.client_id+'_'+self.getTime()
            start = {
                "orderId":self.orderId,
                "room":room,
                "startTime":startTime
            }
            js = json.dumps(start)

            self.orderSendResult = False
            if self.connectionState is True:
                try:
                    self.client.publish(self.orderPubTopic, payload=js, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish starting order report Error')
                else:
                    self.pubMessageState = True
            rospy.sleep(self.TH_cmd_repeat_timer)
            cnt_DeliverReport = 0
            while(self.orderSendResult is False):
                # rospy.loginfo("waiting for start orderSendResult")
                cnt_DeliverReport+=1
                if cnt_DeliverReport>=self.TH_mqtt_request_cnt:
                    self.flag_mqtt_comm_fail_pub=1#1：发布mqtt数据通信异常状态
                    cnt_DeliverReport=self.TH_mqtt_request_cnt

                if self.connectionState is True:
                    try:
                        self.client.publish(self.orderPubTopic, payload=js, qos=1)
                    except socket.error:
                        self.pubMessageState = False
                        rospy.loginfo('Publish starting order report Error')
                    else:
                        self.pubMessageState = True
                rospy.sleep(self.TH_cmd_repeat_timer)            
                if self.orderSendResult is True or self.ob_error_status!=0:
                    if cnt_DeliverReport==self.TH_mqtt_request_cnt:
                        self.flag_mqtt_comm_fail_pub=2#2：发布mqtt数据通信正常状态
                    break
            # if self.orderSendResult is True:
            #     rospy.loginfo("Publish starting order report succeed")
        
    def endDeliverReport(self, pick, endTime):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            # orderId = self.client_id+'_'+self.getTime()
            end = {
                "orderId":self.orderId,
                "endTime":endTime,
                "pickupTime":pick
            }
            js = json.dumps(end)

            self.orderSendResult = False
            if self.connectionState is True:
                try:
                    self.client.publish(self.orderPubTopic, payload=js, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish end order report Error')
                else:
                    self.pubMessageState = True
            rospy.sleep(self.TH_cmd_repeat_timer)
            cnt_DeliverReport = 0
            while(self.orderSendResult is False):
                # rospy.loginfo("waiting for end orderSendResult")
                cnt_DeliverReport+=1
                if cnt_DeliverReport>=self.TH_mqtt_request_cnt:
                    self.flag_mqtt_comm_fail_pub=1#1：发布mqtt数据通信异常状态
                    cnt_DeliverReport=self.TH_mqtt_request_cnt

                if self.connectionState is True:
                    try:
                        self.client.publish(self.orderPubTopic, payload=js, qos=1)
                    except socket.error:
                        self.pubMessageState = False
                        rospy.loginfo('Publish end order report Error')
                    else:
                        self.pubMessageState = True
                rospy.sleep(self.TH_cmd_repeat_timer)            
                if self.orderSendResult is True or self.ob_error_status!=0:
                    if cnt_DeliverReport==self.TH_mqtt_request_cnt:
                        self.flag_mqtt_comm_fail_pub=2#2：发布mqtt数据通信正常状态
                    break
            # if self.orderSendResult is True:
            #     rospy.loginfo("Publish end order report succeed")


    def getTime(self):
        dt = datetime.now()
        return str(dt.year)+str(dt.month)+str(dt.day)+str(dt.hour)+str(dt.minute)

    #从ROS节点py中传递过来机器人的状态变量，后续如果需要传输其他变量，可以在该函数进行添加
    def getRobotStateFromRobot(self,ob_error_status,nav_step):

        self.ob_error_status = ob_error_status
        self.nav_step = nav_step       

    #发送机器人调试信息话题
    def pubDebuginfo(self,Debuginfo_1,Debuginfo_2,Debuginfo_3,Debuginfo_4):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            state = {
                "Debuginfo_1":str(Debuginfo_1),
                "Debuginfo_2":str(Debuginfo_2),
                "Debuginfo_3":str(Debuginfo_3),
                "Debuginfo_4":str(Debuginfo_4),
            }
            js_state = json.dumps(state)

            if self.connectionState is True:
                try:
                    self.client.publish(self.debuginfoPubTopic, payload=js_state, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish Robot Debug info error')
                else:
                    self.pubMessageState = True
                    # rospy.loginfo('Publish Robot Debug info success')


    def pubRobotState(self,ele_occ_status,floor_occ_status,avoidpoint_occ_status,working_status):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            self.ele_occ_status = ele_occ_status
            self.floor_occ_status = floor_occ_status
            self.avoidpoint_occ_status = avoidpoint_occ_status
            self.robot_working_status = working_status
            state = {
                "status":ele_occ_status,
                "working":working_status,
                "floor":str(floor_occ_status),
                "battery":str(0),
                "avoidpoint":str(avoidpoint_occ_status),
                "warehouse":str(0),
                "x":0,
                "y":0
            }
            js_state = json.dumps(state)

            self.robotstatusPub.publish(self.robot_working_status)

            self.robotPubResult = False
            if self.connectionState is True:
                try:
                    self.client.publish(self.statePubTopic, payload=js_state, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish Robot State Error')
                else:
                    self.pubMessageState = True
            rospy.sleep(self.TH_cmd_repeat_timer)
            cnt_pubRobotState = 0
            while(self.robotPubResult is False):
                # rospy.loginfo("waiting for robotPubResult")
                cnt_pubRobotState+=1
                if cnt_pubRobotState>=self.TH_mqtt_request_cnt:
                    self.flag_mqtt_comm_fail_pub=1#1：发布mqtt数据通信异常状态
                    cnt_pubRobotState=self.TH_mqtt_request_cnt

                if self.connectionState is True:
                    try:
                        self.client.publish(self.statePubTopic, payload=js_state, qos=1)
                    except socket.error:
                        self.pubMessageState = False
                        rospy.loginfo('Publish Robot State Error')
                    else:
                        self.pubMessageState = True
                rospy.sleep(self.TH_cmd_repeat_timer)

                if self.robotPubResult is True or self.ob_error_status!=0:
                    if cnt_pubRobotState==self.TH_mqtt_request_cnt:
                        self.flag_mqtt_comm_fail_pub=2#2：发布mqtt数据通信正常状态
                    break
            # if self.robotPubResult is True:
            #     rospy.loginfo("pubRobotState succeed")
        

    def getRobotState(self, msg='[]'):
        if self.robot_for_education == False: #科教版机器人功能标识
 
            #对获取机器人状态函数进行了优化，增加了是否获取成功的判断和重发机制
            request = msg
            rospy.loginfo("---getRobotState---")
            self.getRobotState_success = False
            if self.connectionState is True:
                try:
                    self.client.publish(self.checkRobotPubTopic, payload=request, qos=1)
                except socket.error:
                    self.pubMessageState = False
                    rospy.loginfo('Publish getRobotState Error')
                else:
                    self.pubMessageState = True
            rospy.sleep(self.TH_cmd_repeat_timer)
            cnt_getRobotState = 0
            while(self.getRobotState_success is False):
                # rospy.loginfo("waiting for getRobotState_success")
                cnt_getRobotState+=1
                if cnt_getRobotState>=self.TH_mqtt_request_cnt:
                    self.flag_mqtt_comm_fail_pub=1#1：发布mqtt数据通信异常状态
                    cnt_getRobotState=self.TH_mqtt_request_cnt

                if self.connectionState is True:
                    try:
                        self.client.publish(self.checkRobotPubTopic, payload=request, qos=1)
                    except socket.error:
                        self.pubMessageState = False
                        rospy.loginfo('Publish getRobotState Error')
                    else:
                        self.pubMessageState = True
                rospy.sleep(self.TH_cmd_repeat_timer)
                if self.getRobotState_success is True or self.ob_error_status!=0:
                    if cnt_getRobotState==self.TH_mqtt_request_cnt:
                        self.flag_mqtt_comm_fail_pub=2#2：发布mqtt数据通信正常状态
                    break

            # if self.getRobotState_success is True:
            #     rospy.loginfo("getRobotState succeed")  

    def isFloorOccupied(self, floor):
        if str(floor) in self.floorOccupiedSet:
            # rospy.loginfo('floorOccupiedSet : True')
            return True
        else:
            # rospy.loginfo('floorOccupiedSet : False')
            return False

    def isEleOccupied(self):
        if self.waitingForEle in self.statusSet or self.comingEle in self.statusSet or \
           self.takingEle in self.statusSet or self.getOutEle in self.statusSet:
            self.eleOccupiedState = True
        else:
            self.eleOccupiedState = False
        # rospy.loginfo('eleOccupiedState :' + str(self.eleOccupiedState))
        return self.eleOccupiedState

    def isAvoidOccupied(self, floor):
        if str(floor) in self.avoidOccupiedSet:
            # rospy.loginfo('avoidOccupiedSet : True')
            return True
        else:
            # rospy.loginfo('avoidOccupiedSet : False')
            return False

    def getRobotFloor(self, name):
        for index, value in enumerate(self.robotList):
            if self.robotList[index]['serialId'] == name:
                return self.robotList[index]['floor']

    def loop_start(self):
        self.client.loop_start()

    def loop_stop(self):
        self.client.loop_stop()

    def loop_forever(self):
        self.client.loop_forever()

if __name__ == '__main__':

    robot = Robot_mqtt('robot_test_2')
    robot.checkNetwork()
    print('123213123213213213213')
    robot.connectToServer()
    robot.loop_forever()
    time.sleep(2)
    robot.loop_stop()
    robot.closeConnectToServer()