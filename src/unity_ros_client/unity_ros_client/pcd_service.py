import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger  # ROS 2 표준 트리거 서비스 임포트


class PcdServicePublisher(Node):
    def __init__(self, pcd_file_path):
        super().__init__("pcd_service_publisher_node")

        # 1. 퍼블리셔는 그대로 유지 (Unity의 Subscriber로 보내기 위함)
        self.publisher_ = self.create_publisher(PointCloud2, "pcd_cloud", 10)
        self.pcd_file_path = pcd_file_path
        self.every_k_points = 100

        # 2. 노드 실행 시 PCD 파일을 미리 읽고 변환해 둡니다 (호출 시 딜레이 제거)
        self.msg = self.prepare_pcd_message()

        if self.msg:
            # 3. 타이머를 지우고 '서비스 서버'를 생성합니다.
            # 서비스 이름은 'publish_pcd_trigger' 입니다.
            self.srv = self.create_service(
                Trigger, "publish_pcd_trigger", self.trigger_callback
            )

            self.get_logger().info("==================================================")
            self.get_logger().info(
                "대기 중... 서비스가 호출될 때만 PCD 데이터를 Publish 합니다."
            )
            self.get_logger().info(
                "테스트 명령어: ros2 service call /publish_pcd_trigger std_srvs/srv/Trigger"
            )
            self.get_logger().info("==================================================")

    def prepare_pcd_message(self):
        self.get_logger().info(f"[{self.pcd_file_path}] 파일 읽고 변환 준비 중...")
        try:
            pcd = o3d.io.read_point_cloud(self.pcd_file_path)
            if pcd.is_empty():
                self.get_logger().error("PCD 파일이 비어있습니다.")
                return None

            # 다운샘플링 (균일)
            pcd = pcd.uniform_down_sample(every_k_points=self.every_k_points)

            points = np.asarray(pcd.points, dtype=np.float32)
            colors = np.asarray(pcd.colors) * 255.0
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

    def trigger_callback(self, request, response):
        """
        서비스가 호출(Call)될 때마다 딱 1번씩만 실행되는 콜백 함수
        """
        # 현재 시간으로 스탬프 업데이트 후 퍼블리시
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg)

        self.get_logger().info(
            "요청(Trigger)을 받아 PCD 데이터를 1회 Publish 했습니다!"
        )

        # 요청한 클라이언트에게 '성공' 응답을 보냄
        response.success = True
        response.message = "PCD published successfully to 'pcd_cloud' topic!"
        return response


def main(args=None):
    rclpy.init(args=args)
    # 실제 파일 경로로 수정
    path_list = [
        "/home/jinju/HJ/src/data/240910_noiseX.pcd",
        "/home/jinju/HJ/src/data/241108.pcd",
        "/home/jinju/HJ/src/data/global_241127.pcd",
        "/home/jinju/HJ/src/data/segmented_240910_noiseX.pcd",
    ]
    pcd_file_path = path_list[2]
    node = PcdServicePublisher(pcd_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
