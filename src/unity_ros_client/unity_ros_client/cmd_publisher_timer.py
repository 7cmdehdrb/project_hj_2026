#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter import ttk
import threading
import time
import math


class RosCmdGuiTimer(Node):
    def __init__(self):
        super().__init__("ros_cmd_gui_timer")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "/joint_states__", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/cmd_pose", 10)

        # Joint Names for UR5e
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.setup_gui()

        # Timer for continuous publishing (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS 2 Continuous Command GUI")
        self.root.geometry("450x700")

        # --- Joint State Section ---
        tk.Label(
            self.root,
            text="Joint State Control (Continuous)",
            font=("Arial", 12, "bold"),
        ).pack(pady=10)

        self.joint_sliders = []
        for name in self.joint_names:
            frame = tk.Frame(self.root)
            frame.pack(fill="x", padx=20)

            tk.Label(frame, text=name, width=20, anchor="w").pack(side="left")

            # Slider from -3.14 to 3.14 radians
            slider = tk.Scale(
                frame, from_=-3.14, to=3.14, resolution=0.01, orient="horizontal"
            )
            slider.set(0.0)
            slider.pack(side="right", expand=True, fill="x")
            self.joint_sliders.append(slider)

        ttk.Separator(self.root, orient="horizontal").pack(fill="x", pady=20)

        # --- Pose Section ---
        tk.Label(
            self.root, text="Pose Control (X, Y, Theta)", font=("Arial", 12, "bold")
        ).pack(pady=10)

        # X Slider
        x_frame = tk.Frame(self.root)
        x_frame.pack(fill="x", padx=20)
        tk.Label(x_frame, text="X (m):", width=10, anchor="w").pack(side="left")
        self.slider_x = tk.Scale(
            x_frame, from_=-5.0, to=5.0, resolution=0.01, orient="horizontal"
        )
        self.slider_x.set(0.0)
        self.slider_x.pack(side="right", expand=True, fill="x")

        # Y Slider
        y_frame = tk.Frame(self.root)
        y_frame.pack(fill="x", padx=20)
        tk.Label(y_frame, text="Y (m):", width=10, anchor="w").pack(side="left")
        self.slider_y = tk.Scale(
            y_frame, from_=-5.0, to=5.0, resolution=0.01, orient="horizontal"
        )
        self.slider_y.set(0.0)
        self.slider_y.pack(side="right", expand=True, fill="x")

        # Theta Slider
        theta_frame = tk.Frame(self.root)
        theta_frame.pack(fill="x", padx=20)
        tk.Label(theta_frame, text="Theta (deg):", width=10, anchor="w").pack(
            side="left"
        )
        self.slider_theta = tk.Scale(
            theta_frame, from_=-180.0, to=180.0, resolution=1.0, orient="horizontal"
        )
        self.slider_theta.set(0.0)
        self.slider_theta.pack(side="right", expand=True, fill="x")

        tk.Label(self.root, text="Publishing at 10Hz...", fg="gray").pack(pady=20)

    def timer_callback(self):
        self.publish_joints()
        self.publish_pose()

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(s.get()) for s in self.joint_sliders]
        self.joint_pub.publish(msg)

    def publish_pose(self):
        x = float(self.slider_x.get())
        y = float(self.slider_y.get())
        theta_deg = float(self.slider_theta.get())

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

    def run(self):
        # Run ROS 2 spin in a separate thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        ros_thread.start()

        # Start Tkinter main loop
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    gui = RosCmdGuiTimer()
    gui.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
