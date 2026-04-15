import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import rtde_control, rtde_receive
import numpy as np
import tf_transformations
from tf2_ros import Buffer, TransformListener


class QuestTeleopNode(Node):
    def __init__(self):
        super().__init__("quest_teleop_node")

        # ======================
        # UR 연결
        # ======================
        self.declare_parameter("robot_ip", "192.168.56.101")
        ip = self.get_parameter("robot_ip").value

        try:
            self.rtde_c = rtde_control.RTDEControlInterface(ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        except Exception as e:
            self.get_logger().error(f"RTDE 연결 실패: {e}")
            raise RuntimeError

        self.get_logger().info("UR Connected")

        # ======================
        # 상태 변수
        # ======================
        self.is_tracking = False

        self.start_ctrl_pos = None
        self.start_ctrl_ori = None
        self.start_robot_pose = None

        self.latest_target_pose = None
        self.filtered_target_pose = None
        
        # ======================
        # TF Setup
        # ======================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.R_xr_to_ur = np.eye(3)

        # ======================
        # Sub
        # ======================
        self.create_subscription(Joy, "/quest/right/joy", self.joy_cb, 10)
        self.create_subscription(PoseStamped, "/quest/right/pose", self.pose_cb, 10)

        # ======================
        # 125Hz 제어 루프
        # ======================
        self.timer = self.create_timer(0.008, self.control_loop)

    # ======================
    # Joy
    # ======================
    def joy_cb(self, msg: Joy):
        if len(msg.buttons) <= 5:
            return

        grip = msg.buttons[5] == 1

        if grip and not self.is_tracking:
            self.get_logger().info("START tracking")

            self.is_tracking = True
            self.start_ctrl_pos = None
            self.start_ctrl_ori = None
            self.start_robot_pose = self.rtde_r.getActualTCPPose()
            self.filtered_target_pose = None
            self.R_xr_to_ur = np.eye(3)

            # 그립 순간의 TF를 조회하여 기준 방향 고정
            try:
                # ur5e Base <- xr_origin 변환 획득
                t = self.tf_buffer.lookup_transform('ur5e', 'xr_origin', rclpy.time.Time())
                q = [
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ]
                R_raw = tf_transformations.quaternion_matrix(q)[:3, :3]
                
                # 사용자 요청: 설정 문제로 180도 돌아가 있어서 수동으로 180도(Z축 기준) 회전 보정
                R_180_z = np.array([
                    [-1.0,  0.0,  0.0],
                    [ 0.0, -1.0,  0.0],
                    [ 0.0,  0.0,  1.0]
                ])
                self.R_xr_to_ur = R_raw @ R_180_z
                
                self.get_logger().info("TF XR to UR 매핑 성공! 시점이동(모바일 베이스)이 실시간 반영됩니다. (180도 보정 적용됨)")
            except Exception as e:
                self.get_logger().warn(f"TF 획득 실패 (ur5e <- xr_origin). 기본 방위(1:1) 매핑 사용. 에러: {e}")

        elif not grip and self.is_tracking:
            self.get_logger().info("STOP tracking")

            self.is_tracking = False
            self.latest_target_pose = None
            self.filtered_target_pose = None

            try:
                self.rtde_c.servoStop()
            except:
                pass

    # ======================
    # Pose callback (계산만)
    # ======================
    def pose_cb(self, msg: PoseStamped):
        if not self.is_tracking:
            return

        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        ori = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )

        # 기준 저장
        if self.start_ctrl_pos is None:
            self.start_ctrl_pos = pos
            self.start_ctrl_ori = ori
            return

        if self.start_robot_pose is None:
            return

        # ======================
        # Position
        # ======================
        scaling = 1.0
        delta_xr = (pos - self.start_ctrl_pos) * scaling
        
        # VR(xr_origin) 공간에서의 조작을 로봇 베이스(ur5e) 기준으로 회전 변환
        delta_ur = self.R_xr_to_ur @ delta_xr

        # ======================
        # Orientation
        # ======================
        R0 = tf_transformations.quaternion_matrix(self.start_ctrl_ori)
        R1 = tf_transformations.quaternion_matrix(ori)
        # VR(xr_origin) 로컬 회전 변화량
        R_delta_xr = R1 @ np.linalg.inv(R0)
        
        # 3x3 회전 행렬을 4x4로 확장하여 차원 오류 방지
        R_xr_to_ur_4x4 = np.eye(4)
        R_xr_to_ur_4x4[:3, :3] = self.R_xr_to_ur
        
        # 기저 변환을 통해 회전 변화량을 로봇(ur5e)의 관점으로 맞춰줍니다.
        R_delta_ur = R_xr_to_ur_4x4 @ R_delta_xr @ np.linalg.inv(R_xr_to_ur_4x4)

        robot_aa = self.start_robot_pose[3:6]
        ang = np.linalg.norm(robot_aa)

        if ang < 1e-6:
            R_robot = tf_transformations.identity_matrix()
        else:
            R_robot = tf_transformations.rotation_matrix(ang, robot_aa / ang)

        R_target = R_delta_ur @ R_robot

        angle, direction, _ = tf_transformations.rotation_from_matrix(R_target)
        aa = np.array(direction) * angle

        # ======================
        # pose 구성
        # ======================
        target_pose = [
            float(self.start_robot_pose[0] + delta_ur[0]),
            float(self.start_robot_pose[1] + delta_ur[1]),
            float(self.start_robot_pose[2] + delta_ur[2]),
            float(aa[0]),
            float(aa[1]),
            float(aa[2]),
        ]

        # workspace 제한
        target_pose[2] = np.clip(target_pose[2], 0.05, 0.6)

        # ======================
        # 안전 타겟 검사 (IK 팝업 원천 차단 + 관절 한계 C150A0 방어)
        # ======================
        actual_q = self.rtde_r.getActualQ()

        # 스크립트가 정지된 상태면 IK 계산 함수 자체가 RuntimeError를 발생시킵니다.
        if not hasattr(self, "rtde_c") or not self.rtde_c.isProgramRunning():
            return

        try:
            # 1. 1차 검사: 역운동학 해가 존재하는지부터 확인하여 팝업 에러 방지
            has_solution = self.rtde_c.getInverseKinematicsHasSolution(
                target_pose,
                qnear=actual_q,
                max_position_error=1e-3,
                max_orientation_error=1e-3,
            )

            if not has_solution:
                return  # 원천적으로 도달 불가능한 자세면 타겟 갱신 안 함

            # 2. 2차 검사: 해가 존재하면 그 때 관절 각도를 뽑아와서 안전 한계(Protective Stop) 검사
            q_target = self.rtde_c.getInverseKinematics(
                target_pose,
                qnear=actual_q,
                max_position_error=1e-3,
                max_orientation_error=1e-3,
            )

            if q_target:
                # 조인트가 물리적 한계에 접근하면 C150A0 에러가 뜨므로 파이썬에서 미리 무시
                if not self.rtde_c.isJointsWithinSafetyLimits(q_target):
                    self.get_logger().warn(
                        "Protective Stop 예방: 조인트가 물리적 한계에 너무 가깝습니다.",
                        throttle_duration_sec=1.0,
                    )
                    return

                # 특이점 및 기계 보호: 팔꿈치(Joint 2)와 손목(Joint 4)이 완전히 일자가 되거나 꺾이지 않도록 방어
                if abs(q_target[2]) < 0.35 or abs(q_target[4]) < 0.2:
                    self.get_logger().warn(
                        "Protective Stop 예방: 로봇 팔이 너무 일자로 펴짐/접힘",
                        throttle_duration_sec=1.0,
                    )
                    return

                # C403A0 셀프 충돌 방어: 손목1(q[3])이 하박(Lower Arm) 쪽으로 완전히 뒤집혀 접히는 것 방지.
                q3_mod = abs(q_target[3]) % (2 * 3.14159)
                if abs(q3_mod - 3.14159) < 0.8:
                    self.get_logger().warn(
                        "Protective Stop 예방: 손목이 하박(Lower Arm)과 충돌합니다(C403A0 위험).",
                        throttle_duration_sec=1.0,
                    )
                    return

                # Configuration Flip 방지: 현재 자세에서 관절 해가 비정상적으로 크게 점프하는 경우 필터링
                q_diff = np.max(np.abs(np.array(q_target) - np.array(actual_q)))
                if q_diff > 0.5:
                    self.get_logger().warn(
                        "Protective Stop 예방: IK 계산값이 현재 자세에서 너무 멀리 튀었습니다(자세 뒤집힘 위험).",
                        throttle_duration_sec=1.0,
                    )
                    return

        except Exception as e:
            # 만약 IK 계산 중(예: Protective Stop 상태) 에러가 발생하면 무시합니다.
            return

        # 모든 안전 검사를 통과한 안전한 자센 경우에만 타겟 업데이트
        self.latest_target_pose = target_pose

    # ======================
    # 125Hz control loop
    # ======================
    def control_loop(self):
        if not self.is_tracking or self.latest_target_pose is None:
            return

        try:
            # Low-pass filter (Exponential Moving Average) 적용
            if (
                not hasattr(self, "filtered_target_pose")
                or self.filtered_target_pose is None
            ):
                self.filtered_target_pose = np.array(
                    self.latest_target_pose, dtype=float
                )
            else:
                alpha = 0.15  # 필터 계수: 낮을수록 부드럽지만 딜레이(지연)가 생깁니다. (보통 0.1 ~ 0.3 추천)
                self.filtered_target_pose = (
                    alpha * np.array(self.latest_target_pose, dtype=float)
                    + (1.0 - alpha) * self.filtered_target_pose
                )

            # servoL을 사용하여 test_servo.py 처럼 완벽히 부드러운 Cartesian 이동 구현
            # servoL(pose, velocity, acceleration, dt, lookahead_time, gain)
            # gain 300으로 타겟을 딜레이 없이 단단하게(crisp) 쫓아감
            self.rtde_c.servoL(
                self.filtered_target_pose.tolist(), 0.2, 0.2, 0.008, 0.1, 300
            )
        except Exception as e:
            self.get_logger().error(f"servoL error: {e}")

    # ======================
    # 종료
    # ======================
    def destroy_node(self):
        try:
            self.rtde_c.servoStop()
            self.rtde_c.disconnect()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QuestTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
