#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter import ttk
import threading
import time


class RosCmdGui(Node):
    def __init__(self):
        super().__init__("ros_cmd_gui")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "/cmd_joint_state", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/cmd_pose", 10)

        # Joint Names for UR5e (using link names as mapped in Unity)
        self.joint_names = [
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
        ]

        self.setup_gui()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS 2 Command GUI")
        self.root.geometry("400x600")

        # --- Joint State Section ---
        tk.Label(
            self.root, text="Joint State Control", font=("Arial", 12, "bold")
        ).pack(pady=10)

        self.joint_sliders = []
        for name in self.joint_names:
            frame = tk.Frame(self.root)
            frame.pack(fill="x", padx=20)

            tk.Label(frame, text=name, width=20, anchor="w").pack(side="left")

            # Slider from -3.14 to 3.14 radians (approx)
            slider = tk.Scale(
                frame, from_=-3.14, to=3.14, resolution=0.01, orient="horizontal"
            )
            slider.set(0.0)
            slider.pack(side="right", expand=True, fill="x")
            self.joint_sliders.append(slider)

        tk.Button(
            self.root,
            text="Publish Joint States",
            command=self.publish_joints,
            bg="lightblue",
        ).pack(pady=10)

        ttk.Separator(self.root, orient="horizontal").pack(fill="x", pady=10)

        # --- Pose Section ---
        tk.Label(
            self.root, text="Pose Control (X, Y, Theta)", font=("Arial", 12, "bold")
        ).pack(pady=10)

        pose_frame = tk.Frame(self.root)
        pose_frame.pack(pady=5)

        tk.Label(pose_frame, text="X (m):").grid(row=0, column=0)
        self.entry_x = tk.Entry(pose_frame, width=10)
        self.entry_x.insert(0, "0.0")
        self.entry_x.grid(row=0, column=1, padx=5)

        tk.Label(pose_frame, text="Y (m):").grid(row=0, column=2)
        self.entry_y = tk.Entry(pose_frame, width=10)
        self.entry_y.insert(0, "0.0")
        self.entry_y.grid(row=0, column=3, padx=5)

        tk.Label(pose_frame, text="Theta (deg):").grid(row=1, column=0)
        self.entry_theta = tk.Entry(pose_frame, width=10)
        self.entry_theta.insert(0, "0.0")
        self.entry_theta.grid(row=1, column=1, padx=5, pady=5)

        tk.Button(
            self.root, text="Publish Pose", command=self.publish_pose, bg="lightgreen"
        ).pack(pady=10)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(s.get()) for s in self.joint_sliders]

        self.joint_pub.publish(msg)
        self.get_logger().info("Published JointState")

    def publish_pose(self):
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            theta_deg = float(self.entry_theta.get())
        except ValueError:
            self.get_logger().error("Invalid input for pose")
            return

        import math

        theta_rad = math.radians(theta_deg)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # Quaternion for rotation around Z-axis: [0, 0, sin(theta/2), cos(theta/2)]
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(theta_rad / 2.0)
        msg.pose.orientation.w = math.cos(theta_rad / 2.0)

        self.pose_pub.publish(msg)
        self.get_logger().info(f"Published Pose: x={x}, y={y}, theta={theta_deg} deg")

    def run(self):
        # Run ROS 2 spin in a separate thread so it doesn't block Tkinter
        ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        ros_thread.start()

        # Start Tkinter main loop
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    gui = RosCmdGui()
    gui.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
