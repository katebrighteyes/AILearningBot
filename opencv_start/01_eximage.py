from OpenCV_Functions import *

def ImageProcessing(image):
    result = imageCopy(image)
    return result

road_image_01 = "./solidWhiteCurve.jpg"

image = imageRead(road_image_01, cv2.IMREAD_COLOR)

result = ImageProcessing(image)

imageShow("image_color, cv2.WINDOW_NORMAL", result, cv2.WINDOW_NORMAL)


