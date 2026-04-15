#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import threading
import rtde_control, rtde_receive


class URControlNode(Node):
    def __init__(self):
        super().__init__("ur_control_node")

        # Parameters
        self.declare_parameter("robot_ip", "192.168.56.101")
        self.robot_ip = (
            self.get_parameter("robot_ip").get_parameter_value().string_value
        )

        self.get_logger().info(f"Connecting to UR robot at {self.robot_ip}...")

        try:
            # Initialize RTDE Interfaces
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.get_logger().info("Successfully connected to the robot.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to the robot: {str(e)}")
            return

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Timer for 20Hz publishing
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        # Thread for movement to keep the node spinning
        self.move_thread = threading.Thread(
            target=self.run_example_movement, daemon=True
        )
        self.move_thread.start()

    def publish_joint_states(self):
        try:
            actual_q = self.rtde_r.getActualQ()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # Correct order from base to tip
            msg.name = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
            msg.position = actual_q
            self.joint_pub.publish(msg)
        except Exception as e:
            pass

    def run_example_movement(self):
        time.sleep(2.0)
        self.get_logger().info("Starting example movements...")

        # Zero position (Straight up - Singularity prone)
        home_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # target_q = [0.0, -1.57, -1.57, -1.57, 1.57, 0.0]
        target_q = [0.0, -1.57, -1.57, -1.57, 1.57, 0.0]

        try:
            self.get_logger().info("Moving to home position (MoveJ)...")
            self.rtde_c.moveJ(home_q, 0.5, 0.3)

            time.sleep(5.0)

            self.get_logger().info("Moving to target position (MoveJ)...")
            self.rtde_c.moveJ(target_q, 0.5, 0.3)

            time.sleep(1.0)

            # MoveL is now safe because the arm is not fully extended (elbow != 0)
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = list(current_pose)
            target_pose[2] += 0.05  # Move up by 5cm
            self.get_logger().info(f"Moving linearly up 5cm (MoveL)")
            self.rtde_c.moveL(target_pose, 0.1, 0.2)

            self.get_logger().info("Example movement complete.")
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    node = URControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "rtde_c"):
            node.rtde_c.stopScript()
            node.rtde_c.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
