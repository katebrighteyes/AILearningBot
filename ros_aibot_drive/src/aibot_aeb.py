#!/usr/bin/env python3

import rospy
import cv2

import time
import numpy as np

from ros_aibot_core.msg import MsgCtl
from ros_object_detect.msg import DetInfo

speedVal = 0.1
angleVal = 0.0

stopCmd = 0

def driveCtl(speedval, angleval, stopCmd):
    global speedVal
    global angleVal  
    global ctl_pub

    if (speedval >= 1.0):
      speedval=1.0

    msg = MsgCtl()
    msg.speedCmd = speedval
    msg.headingCmd = angleval
    msg.dirCmd  = 1 
    msg.stopCmd  = stopCmd 
    #print("speedval:{} angleval:{} stopCmd:{}".format(speedval,angleval, msg.stopCmd)) 
    ctl_pub.publish(msg) 

def detresCB(data):
    global speedVal
    global stopCmd
    #print("label:{} x1:{} x2:{}".format(data.label, data.x1, data.x2))
    if data.label == "person":
        stopCmd = 1
        print("label:{}".format(data.label))

rospy.init_node('aibot_aeb', disable_signals=True)

det_sub = rospy.Subscriber('detdata', DetInfo, detresCB, queue_size=10)
ctl_pub = rospy.Publisher('aibot_ctl_msg', MsgCtl, queue_size=10)

loop_rate = rospy.Rate(1)

while not rospy.is_shutdown():
    try:
        driveCtl(speedVal, angleVal, stopCmd);
        stopCmd = 0

        loop_rate.sleep()
    except KeyboardInterrupt:  
        print("key int")  
        break  

print("end")
speedVal = 0.0 
stopCmd = 1
driveCtl(speedVal, angleVal, stopCmd);
time.sleep(1.0)


