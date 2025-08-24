import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import heapq
import math
from scipy.spatial.transform import Rotation as R

# Constants
LIMIT_ANGULAR_SPEED = 0.8
LIMIT_LINEAR_SPEED = 0.5

VIDEO_FILENAME = 'output_video.mp4'
VIDEO_FPS = 10
VIDEO_SIZE = (800, 600)

OBSTACLE_THRESHOLD = 1000.0  # Considera valores maiores como obstáculos intransponíveis

def euler_from_quaternion(quat):
    r = R.from_quat([quat[0], quat[1], quat[2], quat[3]])
    return r.as_euler('xyz', degrees=False)

def read_map(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        return [list(map(float, line.strip().split())) for line in lines]

def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def a_star(map_data, start, goal):
    rows, cols = len(map_data), len(map_data[0])
    moves = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # sem diagonais

    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        for dx, dy in moves:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                cell_cost = map_data[neighbor[0]][neighbor[1]]
                if cell_cost >= OBSTACLE_THRESHOLD:
                    continue  # Ignora obstáculos

                move_cost = 1.0  # custo unitário para movimento direto
                new_cost = cost_so_far[current] + move_cost + cell_cost

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

    if goal not in came_from:
        return []

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

def simplify_path(path):
    if not path:
        return []

    simplified = [path[0]]
    current_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])

    for i in range(1, len(path) - 1):
        next_dir = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if next_dir != current_dir:
            simplified.append(path[i])
            current_dir = next_dir

    simplified.append(path[-1])
    return simplified

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)

        self.bridge = CvBridge()
        self.video_writer = cv2.VideoWriter(VIDEO_FILENAME, cv2.VideoWriter_fourcc(*'mp4v'), VIDEO_FPS, VIDEO_SIZE)

        self.initial_pose_offset = (-10.0, 0.0)
        self.current_pose = (self.initial_pose_offset[0], self.initial_pose_offset[1], 0.0)
        self.prev_time = None

        self.kp_dist = 0.8
        self.ki_dist = 0.0
        self.kd_dist = 0.0
        self.previous_dist_error = 0.0

        self.kp_ang = 0.8
        self.ki_ang = 0.0
        self.kd_ang = 0.0
        self.previous_ang_error = 0.0
        self.TS = 0.1

        self.destinations = []
        self.destination_index = 0
        self.counter = 0

        self.map_data = read_map("mapa_potencial.txt")
        self.center = (len(self.map_data[0]) // 2, len(self.map_data) // 2)

        waypoints = [(6, 0), (6, -4), (-4, -4), (-4, -7), (4, -7), (-10, 0)]
        self.build_path_sequence(waypoints)

        self.timer = self.create_timer(0.1, self.move_robot)

    def build_path_sequence(self, waypoints):
        self.destinations = []
        current = waypoints[0]
        for next_wp in waypoints[1:]:
            start_map = (int(self.center[1] - current[1]), int(self.center[0] + current[0]))
            goal_map = (int(self.center[1] - next_wp[1]), int(self.center[0] + next_wp[0]))
            path = a_star(self.map_data, start_map, goal_map)
            real_path = [(col - self.center[0], self.center[1] - row) for row, col in path]
            self.destinations.extend(real_path[1:])  # skip current pos repeated
            current = next_wp

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            resized_image = cv2.resize(cv_image, (800, 600))
            cv2.imshow("Camera View", resized_image)
            self.video_writer.write(resized_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        v = getattr(self, 'last_linear_x', 0.0)
        x, y, _ = self.current_pose
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt

        self.current_pose = (x, y, yaw)

    def odom_callback(self, msg):
        self.last_linear_x = msg.twist.twist.linear.x

    def move_robot(self):
        if self.current_pose is not None and self.destination_index < len(self.destinations):
            curr_x, curr_y, phi = self.current_pose
            goal_x, goal_y = self.destinations[self.destination_index]

            u_x = goal_x - curr_x
            u_y = goal_y - curr_y

            desired_phi = math.atan2(u_y, u_x)
            angular_error = math.atan2(math.sin(desired_phi - phi), math.cos(desired_phi - phi))
            distance_error = math.hypot(u_x, u_y)

            P_dist = distance_error * self.kp_dist
            D_dist = (distance_error - self.previous_dist_error) * self.kd_dist / self.TS
            I_dist = distance_error * self.ki_dist * self.TS
            self.previous_dist_error = distance_error

            P_ang = angular_error * self.kp_ang
            D_ang = (angular_error - self.previous_ang_error) * self.kd_ang / self.TS
            I_ang = angular_error * self.ki_ang * self.TS
            self.previous_ang_error = angular_error

            uk_disp = max(min(P_dist + I_dist + D_dist, LIMIT_LINEAR_SPEED), -LIMIT_LINEAR_SPEED)
            uk_ang = max(min(P_ang + I_ang + D_ang, LIMIT_ANGULAR_SPEED), -LIMIT_ANGULAR_SPEED)

            twist = Twist()
            if distance_error <= 0.1:
                self.velocity_pub.publish(Twist())
                self.get_logger().info(f"Reached point ({goal_x:.2f}, {goal_y:.2f})")
                self.destination_index += 1

                if self.destination_index >= len(self.destinations):
                    self.get_logger().info("Final destination reached. Saving video...")
                    self.video_writer.release()
                    cv2.destroyAllWindows()
                    self.finished = False
                    return

                return

            if abs(angular_error) > math.radians(5):
                twist.linear.x = 0.0
                twist.angular.z = uk_ang
            else:
                twist.linear.x = uk_disp
                twist.angular.z = uk_ang

            self.velocity_pub.publish(twist)

            if self.counter < 10:
                self.counter += 1
            else:
                self.get_logger().info(f"Current pose: x={curr_x:.2f}, y={curr_y:.2f}, phi={phi:.2f}")
                self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.video_writer.release()
        controller.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
