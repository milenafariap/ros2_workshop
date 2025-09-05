#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def apf_algorithm(current_x, current_y, current_yaw,
                  goal_xy, lidar_data, angle_min, angle_increment,
                  k_att=1.0, k_rep=3.0, threshold=2.5, goal_threshold=0.5,
                  v_max=0.4, w_max=0.4, stop_distance=0.35, alpha_heading=0.6,
                  lidar_in_robot_frame=True, prev_heading=0.0):
    dx = goal_xy[0] - current_x
    dy = goal_xy[1] - current_y
    dist_goal = math.hypot(dx, dy)
    if dist_goal <= goal_threshold:
        return 0.0, 0.0, prev_heading

    F_att = np.array([k_att * dx, k_att * dy], dtype=float)
    F_rep = np.zeros(2, dtype=float)
    min_r = float('inf')

    angle = angle_min
    for r in lidar_data:
        if not (np.isfinite(r) and r > 0.0):
            angle += angle_increment
            continue
        min_r = min(min_r, r)
        if r < threshold:
            mag = k_rep * max(0.0, (1.0/r - 1.0/threshold))
            mag = min(mag, 10.0)  # anti-pico
            a = (current_yaw + angle) if lidar_in_robot_frame else angle
            F_rep += -mag * np.array([math.cos(a), math.sin(a)])
        angle += angle_increment

    F = F_att + F_rep
    if np.allclose(F, 0.0, atol=1e-9):
        return 0.0, 0.0, prev_heading

    theta_des = math.atan2(F[1], F[0])
    heading_cmd = alpha_heading * theta_des + (1.0 - alpha_heading) * prev_heading
    angle_diff = (heading_cmd - current_yaw + math.pi) % (2*math.pi) - math.pi

    # perfil de velocidade linear com “freio de proximidade”
    v = v_max
    if min_r < threshold:
        if min_r <= stop_distance:
            v = 0.0
        else:
            frac = (min_r - stop_distance) / (threshold - stop_distance)
            v = max(0.0, min(v_max, v_max * frac))

    w = max(-w_max, min(w_max, angle_diff))
    return v, w, heading_cmd


class APFNode(Node):
    def __init__(self):
        super().__init__('apf_node')

        # ---------- parâmetros ----------

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 3.0)
        self.declare_parameter('threshold', 2.5)
        self.declare_parameter('goal_threshold', 0.5)
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('w_max', 0.4)
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('alpha_heading', 0.6)
        self.declare_parameter('rate', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.goal = (float(self.get_parameter('goal_x').value),
                     float(self.get_parameter('goal_y').value))

        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.threshold = float(self.get_parameter('threshold').value)
        self.goal_threshold = float(self.get_parameter('goal_threshold').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.alpha_heading = float(self.get_parameter('alpha_heading').value)
        self.rate_hz = float(self.get_parameter('rate').value)

        # ---------- estados ----------
        self.have_scan = False
        self.have_odom = False
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.ranges = []
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_heading = 0.0

        # ---------- QoS ----------
        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_default = 10

        # ---------- subs/pubs ----------
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_scan)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_default)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.timer = self.create_timer(1.0/self.rate_hz, self.step)

        self.get_logger().info(
            f'APF on: scan={self.scan_topic}, odom={self.odom_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'goal=({self.goal[0]:.2f},{self.goal[1]:.2f})'
        )

    def on_scan(self, msg: LaserScan):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.ranges = np.array(msg.ranges, dtype=float)
        self.have_scan = True

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw (assumindo movimento planar z-w)
        self.yaw = math.atan2(2.0 * (q.w*q.z), 1 - 2.0 * (q.z*q.z))
        self.have_odom = True

    def step(self):
        if not (self.have_scan and self.have_odom):
            return

        v, w, self.prev_heading = apf_algorithm(
            current_x=self.x, current_y=self.y, current_yaw=self.yaw,
            goal_xy=self.goal, lidar_data=self.ranges,
            angle_min=self.angle_min, angle_increment=self.angle_increment,
            k_att=self.k_att, k_rep=self.k_rep, threshold=self.threshold,
            goal_threshold=self.goal_threshold, v_max=self.v_max, w_max=self.w_max,
            stop_distance=self.stop_distance, alpha_heading=self.alpha_heading,
            lidar_in_robot_frame=True, prev_heading=self.prev_heading
        )

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    node = APFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

