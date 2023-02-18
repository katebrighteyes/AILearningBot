# 2.	지능형 사물인지

## 2.1 ROS Darknet Yolo 노드

Darknet Node

roslaunch ros_object_detect ros_darknet.launch

사람 인지 확인

Ctl-C


## 2.3 ROS TensorRT DetectNet 노드

Trt DetecNet Node

roslaunch ros_object_detect ros_trt_detectnet.launch

사람 인지 확인

Ctl-C


# 3.	사물 인지 자동 긴급 제동

## 3.1 ROS Darknet + AEB

AIbot Darknet Ready

roslaunch ros_aibot_drive aibot_darknet_ready.launch


AEB

rosrun ros_aibot_drive aibot_aeb.py

주행 중 사람이 감지되면 정지 확인

Ctl-C


## 3.2 ROS TRT+ AEB

AIbot TRT DetectNet Ready

roslaunch ros_aibot_drive aibot_trt_ready.launch

AEB

rosrun ros_aibot_drive aibot_aeb.py

주행 중 사람이 감지되면 정지 확인

Ctl-C
 


# 4.	사물 인지 팔로잉

## 4.1 ROS Darknet + Following

AIbot Darknet Ready

roslaunch ros_aibot_drive aibot_darknet_ready.launch

팔로잉

rosrun ros_aibot_drive aibot_following.py

컵을 감지하면 팔로잉 확인
Ctl-C

## 4.2 ROS TRT + Following

AIbot TRT DetectNet Ready

roslaunch ros_aibot_drive aibot_trt_ready.launch

팔로잉

rosrun ros_aibot_drive aibot_following.py

컵을 감지하면 팔로잉 확인
Ctl-C


# 5.	사물 인지 팔로잉 + 충돌 방지

## 5.1 ROS Darknet + Following + 충돌방지

AIbot Darknet Ready

roslaunch ros_aibot_drive aibot_darknet_ready.launch

팔로잉

(파란색 라이다)
roslaunch ros_aibot_drive aibot_following_fca.launch

(검은색 라이다)
roslaunch ros_aibot_drive  aibot_followingC_fca.launch

컵을 감지하면 팔로잉 확인
Ctl-C


## 5.2 ROS TRT + Following + 충돌방지

AIbot Following Ready

roslaunch ros_aibot_drive aibot_trt_ready.launch

팔로잉 + 충돌방지

(파란색 라이다)
roslaunch ros_aibot_drive aibot_following_fca.launch

(검은색 라이다)
roslaunch ros_aibot_drive  aibot_followingC_fca.launch

컵을 감지하면 팔로잉 하지만 근접거리에 물체가 감지되면 정지
Ctl-C
 

# 6.	차선 감지 및 유지

## 6.1 AIbot + Lane Keeping

AIbot Lane Keeping Ready

roslaunch ros_aibot_core aibot_core_ready.launch

ROS OpenCV Lane Keeping

rosrun ros_aibot_drive aibot_lane_keeping.py

차선을 따라 주행하는 것을 확인

Ctl-C


## 6.2 AIbot + ROS Darknet+ Lane Keeping

AIbot Darknet Ready

roslaunch ros_aibot_drive aibot_darknet_ready.launch


ROS OpenCV Lane Keeping

rosrun ros_aibot_drive aibot_lane_keeping2.py

차선을 따라 주행하다가 사람을 인지하면 정지하는 것을 확인

Ctl-C




 
