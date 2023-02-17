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

rosrun ros_aibot_core aibot_ctl_pub 0

우회전하면 Ctl-C


좌회전

rosrun ros_aibot_core aibot_ctl_pub 0

좌회전하면 Ctl-C


