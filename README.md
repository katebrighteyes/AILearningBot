# AILearningBot

# 1.	ROS 라이다 실습, ROS 모터 제어 실습

# 1.1 ROS 모터 제어 실습

모터 준비

roslaunch ros_aibot_core aibot_core_ready.launch

다른 터미널에서 publish 테스트


전진

rosrun ros_aibot_core aibot_ctl_pub 0

전진하면 Ctl-C


정지

rosrun ros_aibot_core aibot_ctl_pub 5

정지하면 Ctl-C


우회전

rosrun ros_aibot_core aibot_ctl_pub 3

우회전하면 Ctl-C


좌회전

rosrun ros_aibot_core aibot_ctl_pub 4

좌회전하면 Ctl-C


# 1.2 OpenCV + 카메라 실습

폴더 이동

cd

cd opencv_start


OpenCV 이미지 파일 보기

python3 01_eximage.py


OpenCV 비디오 파일 보기

python3 02_exvideo.py


OpenCV 카메라 보기

python3 03_camera.py


OpenCV 도로 비디오 파일에서 차선 검출

python3 04_lanedetect.py


OpenCV 트랙 녹화 파일에서 차선 검출


python3 05_lanedetect_video.py


# 1.3 ROS 카메라 실습

## ROS 카메라 예제

rosrun ros_sensor_test camera_node.py 

Ctl-C


# 1.4 ROS 라이다 실습

## ROS 라이다 기본 예제

파란색 라이다

roslaunch ydlidar lidar_view.launch


검은색 라이다

roslaunch camsense_driver view.launch


Ctl-C


## ROS 라이다 응용 예제

파란색 라이다

roslaunch ros_sensor_test lidar_sub_test.launch


검은색 라이다

roslaunch ros_sensor_test lidar_sub_testC.launch

손을 가까이 대서 근접 거리 경고 확인

Ctl-C



