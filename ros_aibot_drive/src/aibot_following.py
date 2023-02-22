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

Width = 640
Height = 480
speed_value = 0.3
turn_gain = 0.8

def driveCtl(speedVal, angleVal, stopCmd):
    #global speedVal
    #global angleVal  
    global ctl_pub

    if (speedVal >= 1.0):
      speedVal=1.0

    msg = MsgCtl()
    msg.speedCmd = speedVal
    msg.headingCmd = angleVal
    msg.dirCmd  = 1 
    msg.stopCmd  = stopCmd 
    print("speedVal:{} angleVal:{} stopCmd:{}".format(speedVal,angleVal, msg.stopCmd)) 
    ctl_pub.publish(msg) 

def detresCB(data):
    global speedVal
    global stopCmd
    global Width
    global angleVal
    #print("label:{} cx:{} cy:{}".format(data.label, data.centerX, data.centerY))
    #if data.label == "cup":
    if data.label == "person":
        stopCmd = 0
        centerX = data.centerX
        centerY = data.centerY
        bWidth = data.bWidth
        bHeight = data.bHeight
        #print("centerX:{} centerY:{} bWidth:{} bHeight:{}".format(centerX, centerY, bWidth, bHeight))
        Hwidth = Width/2.0
        
        goalXval = float(centerX - Hwidth)/Hwidth
        goalYval = float(Height - centerY)/Height
        angleVal = np.arctan2(goalXval, goalYval)
        print("angleVal:{}".format(angleVal))


rospy.init_node('aibot_following', disable_signals=True)

det_sub = rospy.Subscriber('detdata', DetInfo, detresCB, queue_size=10)
ctl_pub = rospy.Publisher('aibot_ctl_msg', MsgCtl, queue_size=10)

#Width = rospy.param_get("/aibot_go2/Width")
#Height = rospy.param_get("/aibot_go2/Height")
Width = 640
Height = 480
print("Width:{} Height:{}".format(Width, Height))

loop_rate = rospy.Rate(1)

while not rospy.is_shutdown():
    try:
        driveCtl(speedVal, angleVal, stopCmd);
        stopCmd = 1

        loop_rate.sleep(); 
    except KeyboardInterrupt:  
        print("key int")  
        break  

print("end")
speedVal = 0.0 
stopCmd = 1
driveCtl(speedVal, angleVal, stopCmd);
time.sleep(1.0)


