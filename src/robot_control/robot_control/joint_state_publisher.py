#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import rtde_receive

class URJointStatePublisher(Node):
    def __init__(self):
        super().__init__("ur_joint_state_publisher")

        # Parameters
        self.declare_parameter("robot_ip", "192.168.56.101")
        self.declare_parameter("publish_rate", 50.0)
        
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.get_logger().info(f"Connecting to UR RTDE at {self.robot_ip}...")

        try:
            # Initialize RTDE Receive Interface
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.get_logger().info("Successfully connected to the robot RTDE.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to the robot: {str(e)}")
            return

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_states)
        
        self.get_logger().info(f"Publishing joint states at {self.publish_rate}Hz")

    def publish_joint_states(self):
        try:
            # Get actual joint positions from the robot
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
            # Avoid flooding logs with connection errors if robot goes offline
            pass

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    node = URJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
