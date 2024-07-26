#!/usr/bin/env python
#encoding=utf-8
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import math
import time
import copy

class AmclMonitor:
    def __init__(self):
        rospy.loginfo("......amcl_error_control.py is running.....")
        self.first_init = True
        self.pre_pose = PoseWithCovarianceStamped()
        self.current_pose = PoseWithCovarianceStamped()
        self.reinitialConvarianceMatrix = PoseWithCovarianceStamped()
        self.yaw_cur = 0.0
        self.pre_yaw_cur = 0.0
        self.pre_pose.pose.pose.position.x = 0
        self.pre_pose.pose.pose.position.y = 0
        self.current_pose.header.frame_id = "map"
        self.current_pose.pose.pose.position.x = 0
        self.current_pose.pose.pose.position.y = 0
        self.changing_current_map = False   #正在接收重定位命令标识符,标识符为true时不进行位置跳变监听
        self.relocationfinish_flag = False   #重定位完成标识符，只有完成才进入位置跳变监听

        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 5)
        self.reinitialConvarianceMatrix.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        # self.initialpose_listenning = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.ob_initialpose_recall)

        rospy.Subscriber('ob_web_tool_runing', Bool, self.ob_web_tool_runing_cb)
        self.ob_web_tool_runing_Flag=False
        rospy.Subscriber('ob_relocation_finish', Bool, self.ob_relocation_finish_cb)
        self.tflistener = tf.TransformListener()
        rospy.Subscriber('ob_changing_current_map', Bool, self.ob_changing_current_map_cb)
        # rospy.loginfo('pos monitor initial finish')
    

    def ob_web_tool_runing_cb(self, data):
        self.ob_web_tool_runing_Flag = data.data

    # #监听是否有收到重定位话题，如果处于部署模式则可以重定位，不触发Amcl定位跳变问题
    # def ob_initialpose_recall(self, data):
    #     if self.ob_web_tool_runing_Flag==True:
    #         self.first_init = True

    def ob_relocation_finish_cb(self, data):
        self.relocationfinish_flag = True

    def ob_changing_current_map_cb(self, data):
        if data.data == False:
            rospy.sleep(2)
        self.changing_current_map = data.data
        rospy.loginfo("ob_changing_current_map_cb: "+str(self.changing_current_map))

    def mainloop(self):

        self.tflistener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(60))
        (_trans, _rot) = self.tflistener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        _euler1 = tf.transformations.euler_from_quaternion(_rot)
        self.yaw_cur = _euler1[2]
        self.current_pose.pose.pose.position.x = _trans[0]
        self.current_pose.pose.pose.position.y = _trans[1]
        self.current_pose.pose.pose.position.z = _trans[2]
        self.current_pose.pose.pose.orientation.x = _rot[0]    # Makes the origin quaternion valid.
        self.current_pose.pose.pose.orientation.y = _rot[1]    # Makes the origin quaternion valid.
        self.current_pose.pose.pose.orientation.z = _rot[2]    # Makes the origin quaternion valid.
        self.current_pose.pose.pose.orientation.w = _rot[3]    # Makes the origin quaternion valid.
        self.current_pose.header.stamp = rospy.Time.now()

        if self.changing_current_map is True:
            self.first_init = True

        if self.changing_current_map is False:
            if self.first_init:
                # at the first time do not check distance
                self.first_init = False
                self.pre_yaw_cur = self.yaw_cur
                self.pre_pose = copy.deepcopy(self.current_pose)
                # rospy.loginfo('self.first_init: %f', self.first_init)
            else:
                # rospy.loginfo('current_pose x: %f, y: %f', self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y)
                # rospy.loginfo('previous_pose x: %f, y: %f', self.pre_pose.pose.pose.position.x, self.pre_pose.pose.pose.position.y)
                distance = math.sqrt((self.current_pose.pose.pose.position.x - self.pre_pose.pose.pose.position.x)**2 + 
                                    (self.current_pose.pose.pose.position.y - self.pre_pose.pose.pose.position.y)**2)
                # rospy.loginfo('Distance: %f', distance)
                if distance >= 0.5:
                    rospy.logerr("location error with position")
                    rospy.loginfo('Fix pos by using previous pose')
                    self.pre_pose.pose.covariance = self.reinitialConvarianceMatrix.pose.covariance #
                    self.initialpose_pub.publish(self.pre_pose) # using /initpose to adjust error pose
                    rospy.sleep(1)
                    rospy.loginfo('pose relocation finish')
                else:
                    angle_error = self.yaw_cur - self.pre_yaw_cur
                    if(angle_error > math.pi):
                        angle_error -= 2 * math.pi
                    if(angle_error < -math.pi):
                        angle_error += 2 * math.pi                            
                    # rospy.loginfo("angle_error:" + str(angle_error) + ",yaw_cur:" + str(self.yaw_cur) + ",pre_yaw_cur:"+ str(self.pre_yaw_cur))
                    if abs(angle_error) > 0.523:  # 30deg
                        rospy.logerr("location error with angle")
                        rospy.loginfo('Fix pos by using previous pose')
                        self.pre_pose.pose.covariance = self.reinitialConvarianceMatrix.pose.covariance
                        self.initialpose_pub.publish(self.pre_pose)
                        rospy.sleep(1)
                        rospy.loginfo('pose relocation finish')
                    else:
                        self.pre_yaw_cur = self.yaw_cur
                        self.pre_pose = copy.deepcopy(self.current_pose)
if __name__ == '__main__':
    rospy.init_node('amcl_error_control')
    amclmonitor = AmclMonitor()
    while not rospy.is_shutdown():    
        if amclmonitor.relocationfinish_flag is True:
            amclmonitor.mainloop()
        rospy.sleep(0.2)
    rospy.spin()



