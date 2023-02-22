#!/usr/bin/env python

import rospy

import jetson.inference
import jetson.utils

from ros_object_detect.msg import DetInfo

speedVal = 0.1
angleVal = 0.0

stopCmd = 0

rospy.init_node('detectnet_node')

net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = jetson.utils.videoSource("csi://0")      # '/dev/video0' for V4L2
display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file

det_pub = rospy.Publisher('detdata', DetInfo, queue_size=10)

while display.IsStreaming():
    try:
        img = camera.Capture()
        detections = net.Detect(img)
        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
        for detection in detections:
            #print(detection)
            if detection.ClassID == 1:
            #if detection.ClassID == 47: 
                msg = DetInfo()
                msg.label = net.GetClassDesc(detection.ClassID)
                msg.centerX = detection.Center[0]
                msg.centerY = detection.Center[1]
                msg.bWidth = detection.Width
                msg.bHeight = detection.Height
                det_pub.publish(msg)

    except KeyboardInterrupt:  
        print("key int")  
        break  

print("end")
