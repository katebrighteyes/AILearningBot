from OpenCV_Functions import *
import os

min_y = 0
prev_min_y = 0
goalX = 0
prev_goalX = 0
goalY = 0
framenum = 0

def splitLines(lines):
    left_x = []
    left_y = []
    right_x = []
    right_y = []
    for line in lines:
        x1 = line[0,0]
        y1 = line[0,1]
        x2 = line[0,2]
        y2 = line[0,3]
        #if (x1 > 110 and x1 < 140) or (x2 > 110 and x2 < 140):
        #    continue
        if(x2-x1)==0:
            continue
        slope = (float)(y2-y1)/(float)(x2-x1)
        #print(slope)
        if abs(slope) < 0.5:
            continue
        #print(slope)
        if slope <= 0:
            #print('left x1:{} x2:{} y1:{} y2:{}'.format(x1,x2, y1, y2))
            if (slope > -0.6) or ((x1 < 130) and (x2 < 130)):
                left_x.append([x1, x2])
                left_y.append([y1, y2])
        else:
            #print('right x1:{} x2:{} y1:{} y2:{}'.format(x1,x2, y1, y2))
            if (slope < 0.6) or ((x1 > 90) and (x2 > 90)):
                right_x.append([x1, x2])
                right_y.append([y1, y2])
    return left_x, left_y, right_x, right_y


def meanPoint(x):
    sum1 = 0
    sum2 = 0
    for x1, x2 in x:
        sum1 += x1
        sum2 += x2
    sum1 = int(float(sum1)/float(len(x)))
    sum2 = int(float(sum2)/float(len(x)))
    return [sum1, sum2]

def medianPoint(x):
    xx = sorted(x)
    return xx[(int)(len(xx)/2)]
    

def interpolate(list_x, list_y, y):
    x1 = list_x[0]
    x2 = list_x[1]
    y1 = list_y[0]
    y2 = list_y[1]
    return int(float(y - y1) * float(x2-x1) / float(y2-y1) + x1)


def lineFitting2(image, left_x, left_y, right_x, right_y):
    global min_y
    global goalX
    global goalY
    global prev_min_y
    global prev_goalX
    result = imageCopy(image)
    height = image.shape[0]
    lx = ly = max_ly = 0
    min_x_right = min_x_left = 0
    left_line = 0
    if(left_x and left_y):
        lx = meanPoint(left_x)
        ly = meanPoint(left_y)
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
        rx = meanPoint(right_x)
        ry = meanPoint(right_y)    
        max_ry = max(ry)
        sloper = (float)(ry[1]-ry[0])/(float)(rx[1]-rx[0])
        if sloper > 3 :
            min_y = int(height*0.55)
        #print('right slope:{} min_y:{} max_ry:{}'.format(sloper, min_y, max_ry))
        min_x_right = interpolate(rx, ry, min_y)
        max_x_right = max(rx) #rx[1] #interpolate(rx, ry, ry)
        if (max_ry > min_y):
            cv2.line(result, (min_x_right, min_y), (max_x_right, max_ry), (0, 0, 255), 3)
        else:
            min_x_right = 0

    if left_line == 1:
        min_x_left = interpolate(lx, ly, min_y)
        max_x_left = min(lx) #lx[1] #interpolate(lx, ly, ly)
        cv2.line(result, (min_x_left, min_y), (max_x_left, max_ly), (0, 0, 255), 3)

    '''
    if(left_x and left_y):
        min_x_left = interpolate(lx, ly, min_y)
        max_x_left = min(lx) #lx[1] #interpolate(lx, ly, ly)
        #print(max_x_left)
        if (max_ly > min_y):
            cv2.line(result, (min_x_left, min_y), (max_x_left, max_ly), (0, 0, 255), 3)
        else:
            min_x_left = 0
    '''    
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

def ImageProcessing(image):
    global framenum
    global min_y
    global goalX
    global goalY
    global prev_min_y
    global prev_goalX

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

    if(str(type(lines)) == "<class 'NoneType'>"):
        return image
    else:
        lx, ly, rx, ry = splitLines(lines)
        result3 = lineFitting2(image, lx, ly, rx, ry)
        print('framenum:{} goalX:{} min_y:{}'.format(framenum, goalX, min_y))
        result3 = drawCircle(result3, (goalX, min_y), 5, (255, 0, 0), -1)
        prev_min_y = min_y
        prev_goalX = goalX

        goalY = (0.5 - min_y/height) / 2.0

        return result3
    

def LaneDetect(imgfile):
    global min_y
    global goalX
    global goalY

    img = imageRead(imgfile, cv2.IMREAD_COLOR)
    result = ImageProcessing(img)

    '''
    angle = 0.0
    angle_last = 0.0

    goalXval = float(goalX - 50.0)/50.0
    goalY = float(min_y - 50.0)/50.0
    goalYval = (0.5 - goalY) / 2.0
    angle = np.arctan2(goalXval, goalYval)

    print('goalX:{}'.format(goalX))
    print('goalXv:{} goalYv:{} -=> angle:{}'.format(goalXval, goalYval, angle))

    speed_gain_value = 0.18
    steering_value = 0.051
    steering_gain_value = 0.03
    steering_dgain_value = 0.0
    steering_bias_value = 0.0

    speed_value = speed_gain_value

    pid = angle * steering_gain_value + (angle - angle_last) * steering_dgain_value
    angle_last = angle

    steering_value = pid + steering_bias_value

    left_motor_value = max(min(speed_value + steering_value, 1.0), 0.0)
    right_motor_value = max(min(speed_value - steering_value, 1.0), 0.0)	

    #print('speed_value:{} steering_value:{} -=> left_motor:{}'.format(speed_value, steering_value, left_motor_value))

    #print('left_motor:{} right_motor:{}'.format(left_motor_value, right_motor_value))
    '''
    imageShow(imgfile, result, cv2.WINDOW_NORMAL)


def Video(openpath, savepath = None):
    global framenum
    cap = cv2.VideoCapture(openpath)
    if cap.isOpened():
        print("Video Opened")
    else:
        print("Video Not Opened")
        print("Program Abort")
        exit()
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    #out = cv2.VideoWriter(savepath, fourcc, fps, (width, height), True)
    #cv2.namedWindow("Input", cv2.WINDOW_GUI_EXPANDED)
    cv2.namedWindow("Output", cv2.WINDOW_GUI_EXPANDED)
    import OpenCV_Functions
    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret:
            # Our operations on the frame come here
            output = ImageProcessing(frame)
            # Write frame-by-frame
            #out.write(output)
            # Display the resulting frame
            #cv2.imshow("Input", frame)
            cv2.imshow("Output", output)
            framenum = framenum+1
        else:
            break
        # waitKey(int(1000.0/fps)) for matching fps of video
        if cv2.waitKey(int(1000.0/fps)) & 0xFF == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    #out.release()
    cv2.destroyAllWindows()
    return

road_video_01 = "./output.avi"

Video(road_video_01, None)

