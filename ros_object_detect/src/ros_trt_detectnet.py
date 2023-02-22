#!/usr/bin/env python

import rospy

import jetson.inference
import jetson.utils

#from std_msgs.msg import Int32

speedVal = 0.1
angleVal = 0.0

stopCmd = 0

rospy.init_node('detectnet_node', disable_signals=True)

net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = jetson.utils.videoSource("csi://0")      # '/dev/video0' for V4L2
display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file

while display.IsStreaming():
    try:
        img = camera.Capture()
        detections = net.Detect(img)
        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
        for detection in detections:
            #print(detection)
            if detection.ClassID == 1: 
                label = net.GetClassDesc(detection.ClassID)
                print(label)

    except KeyboardInterrupt:  
        print("key int")  
        break  

print("end")
