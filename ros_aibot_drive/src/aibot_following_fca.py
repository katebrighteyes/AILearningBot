#!/usr/bin/env python3

import rospy
import cv2

import time
import numpy as np

from ros_aibot_core.msg import MsgCtl
from ros_object_detect.msg import DetInfo
from sensor_msgs.msg import LaserScan

speedVal = 0.1
angleVal = 0.0

bObject = 0
bSafe = 0
stopCmd = 0

Width = 640
Height = 480
speed_value = 0.3
turn_gain = 0.8

ranges = []

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
    global bObject
    global Width
    global angleVal
    #print("label:{} centerX:{} centerY:{}".format(data.label, data.centerX, data.centerY))
    if data.label == "person":
    #if data.label == "cup":
        bObject = 1
        centerX = data.centerX
        centerY = data.centerY
        bWidth = data.bWidth
        bHeight = data.bHeight
        #print("centerX:{} centerY:{} bWidth:{} bHeight:{}".format(centerX, centerY, bWidth, bHeight))
        Hwidth = Width/2.0
        
        goalXval = float(centerX - Hwidth)/Hwidth
        goalYval = float(Height - centerY)/Height
        angleVal = np.arctan2(goalXval, goalYval)
        #print("angleVal:{}".format(angleVal))

def distanceCB(data):
    global ranges
    ranges = data.ranges

def safeDistance(ranges):
    bSafe = 0
    distance = 0
    if( len(ranges) > 90) :
        bSafe = 1
        for f in range(610,720):
            #print("ranges[{}] : {}".format(f, ranges[f]))
            if(ranges[f] <= 0.2 and ranges[f] != 0.0):
                rospy.loginfo("<WARNING> ranges[{}] : {}".format(f, ranges[f]))
                bSafe = 0
                distance = ranges[f]
                break
        if(bSafe == 1):
            for f in range(0,90):
                #print("ranges[{}] : {}".format(f, ranges[f]))
                if(ranges[f] <= 0.2 and ranges[f] != 0.0):
                    rospy.loginfo("<WARNING> ranges[{}] : {}".format(f, ranges[f]))
                    bSafe = 0
                    distance = ranges[f]
                    break
    print("bSafe:{}".format(bSafe))
    return bSafe, distance


rospy.init_node('aibot_following2', disable_signals=True)

det_sub = rospy.Subscriber('detdata', DetInfo, detresCB, queue_size=10)
ctl_pub = rospy.Publisher('aibot_ctl_msg', MsgCtl, queue_size=10)
rospy.Subscriber("/scan", LaserScan, distanceCB, queue_size = 1)

#Width = rospy.param_get("/aibot_go2/Width")
#Height = rospy.param_get("/aibot_go2/Height")
Width = 640
Height = 480
print("Width:{} Height:{}".format(Width, Height))

loop_rate = rospy.Rate(1)

time.sleep(3)

while not rospy.is_shutdown():
    try:
        bSafe, distance = safeDistance(ranges)
        if(bSafe == 0):
            #rospy.loginfo("WARNING !!! distance : {}".format(ranges[0]))
            stopCmd = 1
        else:
            #print("bObject:{}".format(bObject))
            if(bObject == 1):
                print("angleVal:{}".format(angleVal))
                stopCmd = 0

        driveCtl(speedVal, angleVal, stopCmd);
        stopCmd = 1
        bObject = 0

        loop_rate.sleep()
    except KeyboardInterrupt:  
        print("key int")  
        break  

print("end")
speedVal = 0.0 
stopCmd = 1
driveCtl(speedVal, angleVal, stopCmd);
time.sleep(1.0)


