#!/usr/bin/env python3

import rospy
import cv2
from OpenCV_Functions import *

from sensor_msgs.msg import Image
from ros_object_detect.msg import DetInfo
from cv_bridge import CvBridge, CvBridgeError
from ros_aibot_core.msg import MsgCtl

import time

bObject = 0
ObjectVal = 0
bNoLane = 0
prev_bNoLane = 0

bridge = CvBridge()
cv_image = None
   

min_y = 0
prev_min_y = 0
goalX = 0
prev_goalX = 0
framenum = 0

bEnd = 0


speedVal = 0.1
angleVal = 0.0

stopCmd = 0


def img_callback(img_data):
    global bridge
    global cv_image
    global bObject
    global ObjectVal

    cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    
    output = ImageProcessing(cv_image)
    cv2.imshow("Output", output)
    if bObject == 1:
        stopCmd = 1
    else:
        stopCmd = bNoLane

    if bEnd == 0:
        driveCtl(speedVal, angleVal, stopCmd)
    stopCmd = 0  
    bObject= 0
    
def driveCtl(speedval, angleval, stopCmd): 
    global ctl_pub

    if (speedval >= 1.0):
      speedval=1.0

    msg = MsgCtl()
    msg.speedCmd = speedval
    msg.headingCmd = angleval
    msg.dirCmd  = 1 
    msg.stopCmd  = stopCmd 
    print("speedval:{} angleval:{} stopCmd:{}".format(speedval,angleval, msg.stopCmd)) 
    ctl_pub.publish(msg) 

def detresCB(data):
    global bObject
    global ObjectVal
    #print("label:{} x1:{} x2:{}".format(data.label, data.x1, data.x2))
    if data.label == "person":
        bObject = 1
        centerX = data.centerX
        centerY = data.centerY
        bWidth = data.bWidth
        bHeight = data.bHeight        
        print("label:{} bObject:{}".format(data.label, bObject))

def lineFitting2(image, left_x, left_y, right_x, right_y):
    global min_y
    global goalX
    global prev_min_y
    global prev_goalX

    result = imageCopy(image)
    height = image.shape[0]
    lx = ly = max_ly = 0
    min_x_right = min_x_left = 0
    left_line = 0
    if(left_x and left_y):
        lx = getMeanPoint(left_x)
        ly = getMeanPoint(left_y)
        #print('lx:{}'.format(lx))
        
        slope = (float)(ly[1]-ly[0])/(float)(lx[1]-lx[0])
        if slope >= -1 :
            min_y = int(height*0.55)
        max_ly = max(ly)
        #print('left slope:{} min_y:{} max_ly:{}'.format(slope, min_y, max_ly))
        #cv2.line(result, (lx[0], max_ly), (lx[1], min_y), (255, 0, 0), 3)
        #print(max_ly)
        if (max_ly > min_y):
            left_line = 1
        else:
            min_y = int(height*0.4)

    if(right_x and right_y):
        rx = getMeanPoint(right_x)
        ry = getMeanPoint(right_y)    
        max_ry = max(ry)
        sloper = (float)(ry[1]-ry[0])/(float)(rx[1]-rx[0])
        if sloper > 3 :
            min_y = int(height*0.55)
        #print('right slope:{} min_y:{} max_ry:{}'.format(sloper, min_y, max_ry))
        min_x_right = interpolateXY(rx, ry, min_y)
        max_x_right = max(rx) 
        if (max_ry > min_y):
            cv2.line(result, (min_x_right, min_y), (max_x_right, max_ry), (0, 0, 255), 3)
        else:
            min_x_right = 0

    if left_line == 1:
        min_x_left = interpolateXY(lx, ly, min_y)
        max_x_left = min(lx) 
        cv2.line(result, (min_x_left, min_y), (max_x_left, max_ly), (0, 0, 255), 3)
  
    roadWidth = 130
    
    #print('min_x_right:{} min_x_left:{}'.format(min_x_right, min_x_left))
    if min_x_right > 0 and min_x_left > 0:
        roadWidth = min_x_right - min_x_left
        #print('roadWidth:{}'.format(roadWidth))
        goalX = min_x_left + int(roadWidth/2)
    elif min_x_right > 0 and min_x_left == 0:
        goalX = min_x_right - int(roadWidth/2)
        if min_y < int(height*0.55):
            min_y = int(height*0.58)
        else:
            min_y = int(height*0.7)
            roadWidth = 180
    elif min_x_right == 0 and min_x_left > 0:
        goalX = min_x_left + int(roadWidth/2)
        if min_y < int(height*0.55):
            min_y = int(height*0.58)
        else:
            min_y = int(height*0.7)
            roadWidth = 180
    else:
        if (prev_goalX>0):
            goalX = prev_goalX
        if (prev_min_y>0):
            min_y = prev_min_y
        print('prev_min_y:{} prev_goalX:{}'.format(prev_min_y, prev_goalX))

    #print('roadWidth:{} goalX:{} min_y:{}'.format(roadWidth, goalX, min_y))
    return result



def lineFollow(goalX, goalY, width, height):

    angle = 0.0
    angle_last = 0.0

    #goalY = (0.5 - min_y/height) / 2.0

    Hwidth = width/2.0
    
    goalXval = float(goalX - Hwidth)/Hwidth
    goalYval = float(height - goalY)/height
    angle = np.arctan2(goalXval, goalYval)

    return angle

def ImageProcessing(image):
    global framenum
    global min_y
    global goalX
    global goalY
    global prev_min_y
    global prev_goalX

    global speedVal
    global angleVal  
    global bNoLane
    global prev_bNoLane

    result = imageCopy(image)
    image_gray = convertColor(result, cv2.COLOR_BGR2GRAY)
    resultT = imageThreshold(image_gray, 175, 255, cv2.THRESH_BINARY)
    #image_edge = cannyEdge(image_gray, 100, 200)
    height, width = image.shape[:2]
    min_y = int(height*0.4)
    pt1 = (width*0.02, min_y) #height*0.45)
    pt2 = (width*0.98, min_y)
    pt3 = (width*0.98, height*0.70)
    pt4 = (width*0.02, height*0.70)
    roi_corners = np.array([[pt1, pt2, pt3, pt4]], dtype=np.int32)
    result = polyROI(resultT, roi_corners)

    result = cannyEdge(result, 100, 200)
    lines = houghLinesP(result, 1, np.pi/180, 10, 10)
    #result2 = drawHoughLinesP(image, lines)
    #imageShow("HP", result2, cv2.WINDOW_NORMAL)
    #return result2
    #print(type(lines))
    prev_bNoLane = bNoLane

    if(str(type(lines)) == "<class 'NoneType'>"):
        bNoLane = 1
        return image
    else:
        lx, ly, rx, ry = splitLRLines(lines)
        result3 = lineFitting2(image, lx, ly, rx, ry)
        #print('framenum:{} goalX:{} min_y:{}'.format(framenum, goalX, min_y))
        result3 = drawCircle(result3, (goalX, min_y), 5, (255, 0, 0), -1)
        prev_min_y = min_y
        prev_goalX = goalX

        goalY = (0.5 - min_y/height) / 2.0

        angleVal = lineFollow(goalX, min_y, width, height)
        speedVal = 0.1
        bNoLane = 0
        #print('angleVal:{} speedVal:{} bNoLane:{}'.format(angleVal, speedVal, bNoLane))

        return result3
    

rospy.init_node("lane_keeping_node", disable_signals=True)

rospy.Subscriber("csi_image", Image, img_callback)
#bridge = CvBridge()


det_sub = rospy.Subscriber('detdata', DetInfo, detresCB, queue_size=10)
ctl_pub = rospy.Publisher('aibot_ctl_msg', MsgCtl, queue_size=10)

loop_rate = rospy.Rate(1)

fps = 30
width = 360
height = 240

print('width:{} height:{}'.format(width,height))

time.sleep(1.5)

while not rospy.is_shutdown():
    try:
        '''
        output = ImageProcessing(cv_image)
        cv2.imshow("Output", output)
        if bObject == 1:
            stopCmd = 1
            if ObjectVal<1:
                ObjectVal = ObjectVal+1
            else:
                ObjectVal = 0
                bObject = 0
    
        else:
            stopCmd = bNoLane

        driveCtl(speedVal, angleVal, stopCmd)
        stopCmd = 0
        '''
        if cv2.waitKey(int(1000.0/fps)) & 0xFF == ord('q'):
            break

    except KeyboardInterrupt:  
        print("key int")  
        bEnd = 1
        break  

print("end")

speedVal = 0.0 
stopCmd = 1
driveCtl(speedVal, angleVal, stopCmd);
time.sleep(1.0)

print("bEnd")
cv2.destroyAllWindows()

