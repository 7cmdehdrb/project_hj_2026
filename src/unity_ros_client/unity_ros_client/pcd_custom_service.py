import os
import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from unity_ros_interfaces.srv import PcdService


class PcdCustomServicePublisher(Node):
    def __init__(self):
        super().__init__("pcd_custom_service_publisher_node")

        # 1. 퍼블리셔 그대로 유지
        self.publisher_ = self.create_publisher(PointCloud2, "pcd_cloud", 10)

        # PCD 파일 리스트 및 미리 로드 (상대 경로로 설정)
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        data_dir = os.path.join(package_dir, "resource", "data")
        
        self.path_list = [
            os.path.join(data_dir, "240910_noiseX.pcd"),
            os.path.join(data_dir, "241108.pcd"),
            os.path.join(data_dir, "global_241127.pcd"),
            os.path.join(data_dir, "segmented_240910_noiseX.pcd"),
        ]
        self.pcd_objects = self.preload_pcds()

        # 2. 커스텀 서비스 서버 생성
        self.srv = self.create_service(
            PcdService, "publish_pcd_custom", self.service_callback
        )

        self.get_logger().info("==================================================")
        self.get_logger().info("커스텀 PCD 서비스 대기 중 (모든 PCD 로드 완료)")
        self.get_logger().info(
            "테스트 명령어: ros2 service call /publish_pcd_custom unity_ros_interfaces/srv/PcdService \"{trigger: true, index: 0, sampling_value: 100}\""
        )
        self.get_logger().info("==================================================")

    def preload_pcds(self):
        pcds = []
        for path in self.path_list:
            self.get_logger().info(f"PCD 로딩 중: {path}")
            try:
                pcd = o3d.io.read_point_cloud(path)
                if pcd.is_empty():
                    self.get_logger().warning(f"PCD 파일이 비어있습니다: {path}")
                pcds.append(pcd)
            except Exception as e:
                self.get_logger().error(f"PCD 로딩 실패 ({path}): {e}")
                pcds.append(None)
        return pcds

    def prepare_pcd_message(self, pcd, sampling_value):
        if pcd is None or pcd.is_empty():
            self.get_logger().error("유효하지 않은 PCD 데이터입니다.")
            return None

        self.get_logger().info(f"PCD 변환 준비 중 (Sampling: {sampling_value})...")
        try:
            # 원본 PCD를 건드리지 않기 위해 복사 후 다운샘플링
            processed_pcd = pcd
            if sampling_value > 1:
                processed_pcd = pcd.uniform_down_sample(every_k_points=int(sampling_value))

            points = np.asarray(processed_pcd.points, dtype=np.float32)
            colors = np.asarray(processed_pcd.colors) * 255.0
            num_points = len(points)

            r = np.asarray(colors[:, 0], dtype=np.uint32)
            g = np.asarray(colors[:, 1], dtype=np.uint32)
            b = np.asarray(colors[:, 2], dtype=np.uint32)
            rgb_packed = (r << 16) | (g << 8) | (b)
            rgb_float = rgb_packed.view(np.float32)

            cloud_data = np.zeros(
                num_points,
                dtype=[
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("rgb", np.float32),
                ],
            )
            cloud_data["x"] = points[:, 0]
            cloud_data["y"] = points[:, 1]
            cloud_data["z"] = points[:, 2]
            cloud_data["rgb"] = rgb_float

            msg = PointCloud2()
            msg.header = Header()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = 1
            msg.width = num_points
            msg.is_dense = False
            msg.is_bigendian = False

            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step = msg.point_step * msg.width
            msg.data = cloud_data.tobytes()

            self.get_logger().info("ROS 메시지 변환 완료!")
            return msg

        except Exception as e:
            self.get_logger().error(f"변환 오류: {e}")
            return None

    def service_callback(self, request, response):
        """
        커스텀 서비스 콜백
        request.trigger, request.index, request.sampling_value
        """
        self.get_logger().info(
            f"요청 수신: index={request.index}, sampling={request.sampling_value}, trigger={request.trigger}"
        )

        if not request.trigger:
            response.success = False
            response.message = "Trigger is false. Doing nothing."
            return response

        # 1. 인덱스 범위 확인 (-1은 비우기, 0 이상은 로드된 PCD)
        if request.index < -1 or request.index >= len(self.path_list):
            response.success = False
            response.message = f"Invalid index ({request.index}). Valid range is -1 to {len(self.path_list)-1}."
            return response

        # 2. 샘플링 값 확인 (index가 -1이 아닐 때만 필수)
        if request.index != -1 and request.sampling_value <= 0:
            response.success = False
            response.message = f"Invalid sampling_value ({request.sampling_value}). Must be a positive integer (>= 1)."
            return response

        # 인덱스가 -1인 경우 빈 메시지 퍼블리시 (RViz2/Unity 렌더링 부하 제거용)
        if request.index == -1:
            empty_msg = PointCloud2()
            empty_msg.header = Header()
            empty_msg.header.frame_id = "map"
            empty_msg.header.stamp = self.get_clock().now().to_msg()
            empty_msg.height = 1
            empty_msg.width = 0
            empty_msg.is_dense = False
            empty_msg.is_bigendian = False
            # 필드 구조는 유지하되 데이터만 비움 (시각화 툴 호환성)
            empty_msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            empty_msg.point_step = 16
            empty_msg.row_step = 0
            empty_msg.data = b""
            
            self.publisher_.publish(empty_msg)
            response.success = True
            response.message = "Published an empty PointCloud2 message to clear visualization (index -1)."
            return response

        file_path = self.path_list[request.index]
        pcd_obj = self.pcd_objects[request.index]
        msg = self.prepare_pcd_message(pcd_obj, request.sampling_value)

        if msg:
            self.publisher_.publish(msg)
            response.success = True
            response.message = f"Successfully published PCD index {request.index}"
        else:
            response.success = False
            response.message = "Failed to prepare PCD message."

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PcdCustomServicePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
