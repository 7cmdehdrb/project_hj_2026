#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import threading
import rtde_control, rtde_receive


class URServoNode(Node):
    def __init__(self):
        super().__init__("ur_servo_node")
        
        # Parameters
        self.declare_parameter("robot_ip", "192.168.56.101")
        self.robot_ip = (
            self.get_parameter("robot_ip").get_parameter_value().string_value
        )

        self.get_logger().info(f"Connecting to UR robot at {self.robot_ip} (Servo Mode)...")

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
        self.get_logger().info("Starting smooth movements...")

        # Positions
        home_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target_q = [0.0, -1.57, -1.57, -1.57, 1.57, 0.0]

        try:
            self.get_logger().info("Moving to home position (MoveJ)...")
            self.rtde_c.moveJ(home_q, 0.5, 0.3)
            time.sleep(5.0)

            self.get_logger().info("Moving to target position (MoveJ)...")
            self.rtde_c.moveJ(target_q, 0.5, 0.3)
            time.sleep(1.0)

            # Smoothly move up by 5cm using servoL interpolation
            current_pose = self.rtde_r.getActualTCPPose()
            self.get_logger().info(f"Moving linearly up 5cm (servoL interpolation)")
            
            duration = 2.0  # seconds
            dt = 0.008      # 125Hz
            steps = int(duration / dt)
            
            for i in range(steps):
                target_pose = list(current_pose)
                # Linear interpolation for Z axis
                target_pose[2] += 0.05 * (i / steps)
                
                # servoL(pose, velocity, acceleration, lookahead_time, gain)
                self.rtde_c.servoL(target_pose, 0.1, 1.0, dt, 0.1, 300)
                time.sleep(dt)

            self.rtde_c.servoStop()
            self.get_logger().info("Smooth movement complete.")
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    node = URServoNode()

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
