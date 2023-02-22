#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan

ranges_list = []

def callback(data):
    global ranges_list
    ranges_list = data.ranges

def safeDistance(ranges):
    bSafe = 0
    if( len(ranges) > 90) :
        bSafe = 1
        for f in range(680,720):
            #print("ranges[{}] : {}".format(f, ranges[f]))
            if(ranges[f] <= 0.2 and ranges[f] != 0.0):
                bSafe = 0
                rospy.loginfo("<WARNING> ranges[{}] : {}".format(f, ranges[f]))
                break
        if(bSafe == 1):
            for f in range(0,40):
                #print("ranges[{}] : {}".format(f, ranges[f]))
                if(ranges[f] <= 0.2 and ranges[f] != 0.0):
                    bSafe = 0
                    rospy.loginfo("<WARNING> ranges[{}] : {}".format(f, ranges[f]))
                    break
    print("bSafe:{}".format(bSafe))
    return bSafe

rospy.init_node('Lidar_Node')
rospy.Subscriber("/scan", LaserScan, callback, queue_size = 1)

time.sleep(3)

print(len(ranges_list))

while not rospy.is_shutdown():
    #if( len(ranges_list) > 90) :
        #print(len(ranges_list))
    bSafe = safeDistance(ranges_list)
    time.sleep(0.1)
