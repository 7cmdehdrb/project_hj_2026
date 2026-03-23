# Unity ROS Client Package

이 패키지는 Unity 클라이언트와 상호작용하기 위한 ROS2 노드들을 포함하고 있습니다. 로봇 팔(UR5e) 및 모바일 베이스 제어 명령을 발행하고, PCD 데이터를 서비스 형태로 제공합니다.

## 주요 노드 및 기능

### 1. `cmd_publisher`
- **설명**: Tkinter GUI를 통해 로봇 팔의 관절(Joint)과 모바일 베이스의 포즈(Pose) 정보를 수동으로 발행합니다.
- **주요 토픽**:
  - `/cmd_joint_state` (`sensor_msgs/JointState`): 조인트 각도 명령
  - `/cmd_pose` (`geometry_msgs/PoseStamped`): 베이스 포즈 명령
- **특징**: 'Publish' 버튼을 명시적으로 누를 때만 데이터가 발행됩니다.

### 2. `cmd_publisher_timer`
- **설명**: `cmd_publisher`와 동일한 기능을 제공하지만, 타이머를 사용하여 실시간(10Hz)으로 데이터를 발행합니다.
- **특징**: 슬라이더 조작 시 실시간으로 Unity 측에 반영됩니다.

### 3. `pcd_service`
- **설명**: `.pcd` 파일을 읽어 `sensor_msgs/PointCloud2` 형식으로 변환한 뒤, 서비스 요청이 있을 때만 데이터를 발행합니다.
- **서비스 이름**: `/publish_pcd_trigger` (`std_srvs/srv/Trigger`)
- **발행 토픽**: `pcd_cloud` (`sensor_msgs/PointCloud2`)
- **특징**: 네트워크 부하를 줄이기 위해 Service-Trigger 방식을 사용하며, 코드 내부에서 다운샘플링 기능을 포함하고 있습니다.

---

## 설치 및 실행 방법

### 빌드
```bash
colcon build --packages-select unity_ros_client
source install/setup.bash
```

### 실행
각 노드를 실행하려면 아래 명령어를 사용합니다.

```bash
# GUI 커맨드 퍼블리셔 (수동)
ros2 run unity_ros_client cmd_publisher

# GUI 커맨드 퍼블리셔 (실시간/타이머)
ros2 run unity_ros_client cmd_publisher_timer

# PCD 서비스 서버
ros2 run unity_ros_client pcd_service
```

### PCD 데이터 요청 트리거
```bash
ros2 service call /publish_pcd_trigger std_srvs/srv/Trigger
```
