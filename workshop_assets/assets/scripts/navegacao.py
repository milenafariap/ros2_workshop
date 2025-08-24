import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

class ZigZagNavigator(Node):
    def __init__(self, linhas, colunas, espacamento):
        super().__init__('zigzag_navigator')
        self.linhas = linhas
        self.colunas = colunas
        self.espacamento = espacamento

        self.path = self.generate_zigzag_path()
        self.current_target_idx = 0
        self.current_pose = None

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Câmera
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.video_writer = cv2.VideoWriter(
            'output_video.mp4',
            cv2.VideoWriter_fourcc(*'mp4v'),
            10, (800, 600)
        )

    def generate_zigzag_path(self):
        path = []
        for i in range(self.linhas):
            row = range(self.colunas) if i % 2 == 0 else reversed(range(self.colunas))
            for j in row:
                x = j * self.espacamento
                y = i * self.espacamento
                path.append((x, y))
        return path

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw(self):
        orientation = self.current_pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def timer_callback(self):
        if self.current_pose is None or self.current_target_idx >= len(self.path):
            return

        # Posição atual
        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # Alvo atual
        target_x, target_y = self.path[self.current_target_idx]
        dist = self.distance(x, y, target_x, target_y)
        angle_to_target = math.atan2(target_y - y, target_x - x)
        yaw = self.get_yaw()
        angle_diff = angle_to_target - yaw

        twist = Twist()

        if dist < 0.2:
            self.get_logger().info(f"✅ Alvo {self.current_target_idx + 1} atingido")
            self.current_target_idx += 1
            if self.current_target_idx >= len(self.path):
                self.get_logger().info("🏁 Todos os pontos visitados.")
                self.video_writer.release()
                cv2.destroyAllWindows()
                return
        else:
            if abs(angle_diff) > 0.1:
                twist.angular.z = 0.5 * angle_diff
                twist.linear.x = 0.0
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0

            self.publisher.publish(twist)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            resized_image = cv2.resize(cv_image, (800, 600))
            cv2.imshow("Camera View", resized_image)
            self.video_writer.write(resized_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Erro ao processar imagem: {e}")

def main():
    rclpy.init()
    linhas = 4
    colunas = 10
    espacamento = 2.0
    navigator = ZigZagNavigator(linhas, colunas, espacamento)

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
