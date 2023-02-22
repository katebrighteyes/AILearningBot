from OpenCV_Functions import *

min_y = 0
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
        slope = (float)(y2-y1)/(float)(x2-x1)
        if abs(slope) < 0.5:
            continue
        if slope <= 0:
            left_x.append([x1, x2])
            left_y.append([y1, y2])
        else:
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

    
def lineFitting(image, left_x, left_y, right_x, right_y):
    result = imageCopy(image)
    height = image.shape[0]
    lx = meanPoint(left_x)
    ly = meanPoint(left_y)
    rx = meanPoint(right_x)
    ry = meanPoint(right_y)
    min_y = int(height*0.6)
    max_y = height
    min_x_left = interpolate(lx, ly, min_y)
    max_x_left = interpolate(lx, ly, max_y)
    min_x_right = interpolate(rx, ry, min_y)
    max_x_right = interpolate(rx, ry, max_y)
    cv2.line(result, (min_x_left, min_y), (max_x_left, max_y), (0, 0, 255), 3)
    cv2.line(result, (min_x_right, min_y), (max_x_right, max_y), (0, 0, 255), 3)
    return result

def lineFitting2(image, left_x, left_y, right_x, right_y):
    global min_y
    global goalX
    global goalY
    result = imageCopy(image)
    height = image.shape[0]
    if(left_x and left_y):
        lx = meanPoint(left_x)
        ly = meanPoint(left_y)
        #print('lx')
        #print(lx)
        slope = (float)(ly[1]-ly[0])/(float)(lx[1]-lx[0])
        print(slope)
        max_ly = max(ly)
        #print(max_ly)
        min_x_left = interpolate(lx, ly, min_y)
        max_x_left = min(lx) #lx[1] #interpolate(lx, ly, ly)
        #print(max_x_left)
        cv2.line(result, (min_x_left, min_y), (max_x_left, max_ly), (0, 0, 255), 3)
    if(right_x and right_y):
        rx = meanPoint(right_x)
        ry = meanPoint(right_y)    
        max_ry = max(ry)
        min_x_right = interpolate(rx, ry, min_y)
        max_x_right = max(rx) #rx[1] #interpolate(rx, ry, ry)
        cv2.line(result, (min_x_right, min_y), (max_x_right, max_ry), (0, 0, 255), 3)
    #roadWidth = max_x_right - min_x_left
    #goalX = int(roadWidth/2)
    #print('roadWidth:{} goalX:{}'.format(roadWidth, goalX))
    return result


def imageProcessing(image):
    global min_y
    result = imageCopy(image)
    image_gray = convertColor(image, cv2.COLOR_BGR2GRAY)
    #image_edge = cannyEdge(image_gray, 100, 200)
    height, width = image.shape[:2]

    min_y = int(height*0.75)
    pt1 = (width*0.00, min_y) #height*0.45)
    pt2 = (width*1.00, min_y)
    pt3 = (width*1.00, height*0.95)
    pt4 = (width*0.00, height*0.95)	
    roi_corners = np.array([[pt1, pt2, pt3, pt4]], dtype=np.int32)
    result = polyROI(result, roi_corners)

    result = cannyEdge(result, 100, 200)
    lines = houghLinesP(result, 1, np.pi/180, 10, 10)
    lx, ly, rx, ry = splitLines(lines)
    result3 = lineFitting2(image, lx, ly, rx, ry)

    #result = lineFitting(image, lines, (0, 0, 255), 5, 5. * np.pi / 180.)
    return result3

def VideoProcessing(openpath, savepath = "output.avi"):
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
            output = imageProcessing(frame)
            # Write frame-by-frame
            #out.write(output)
            # Display the resulting frame
            #cv2.imshow("Input", frame)
            cv2.imshow("Output", output)
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

road_video_01 = "./solidWhiteRight.mp4"

#VideoProcessing(road_video_01, "output03v.mp4")
VideoProcessing(road_video_01, None)
