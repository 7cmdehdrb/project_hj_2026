# Unity 클라이언트 제작기

## 1. PCD 데이터 띄우기

- ROS2 서버 사용
- ROS2 서버에 Service 요청 보내서 받아오는 구조
- 받아온 PointCloud2를 랜더링

-> 데이터 그대로 사용하면 랜더링 개 느릴 가능성 있음. 필요할 경우 다운샘플링 수행
-> Publish로 주기적으로 하면 네트워크 죽음


## 2. Mobile Base & Manipulator (UR5e) 띄우기

- ROS2 서버 사용
- Mobile Base는 Odometry or PoseStamp 메세지 Subscribe 받아서 "반영"
- Manipulator는 JointState 메세지 Subscribe 받아서 "반영"

## 3. JoyStick 메시지 발행

- 오른쪽, 왼쪽 컨트롤러 메시지 발행 (손 아님, 컨트롤러)
- Joy 메시지 타입 -> 컨트롤러 조이스틱 상태 (버튼 + 조이스틱)
- 컨트롤러 "Pose" 발행 (Player 원점에 대하여 Local Pose)

---

ros2 service call /publish_pcd_trigger std_srvs/srv/Trigger


----
irol wifi연결하면 tcp 통신이 안됨. 

vr기기, 컴퓨터 둘다 hotspot으로 연결하는 편이 나음



##
손 기준을 origin, frame = origin
xr orin - tf -  world

tf 
    world
        -mobile
            -ur5e
        -xr_origin
            -xr_head
            -left_controller
            -right_controller



xr_origin pose publish,
anipulator gripper 다시 추가 wrist3 
ip 수정되게