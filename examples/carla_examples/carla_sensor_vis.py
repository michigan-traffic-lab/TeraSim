import yaml
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import rosbag2_py
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

class SensorVisualizer(Node):
    def __init__(self, config_path="sensor_config.yaml"):
        super().__init__("sensor_visualizer")
        self.bridge = CvBridge()
        self.sub_list = []  # <-- fixed name
        self.image_data = {}
        self.lidar_data = None

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        for sensor in config["sensors"]:
            topic = sensor["topic"]
            s_type = sensor["type"]

            if s_type == "LiDAR":
                self.sub_list.append(
                    self.create_subscription(PointCloud2, topic, self.lidar_callback, 10)
                )
                self.lidar_topic = topic
            else:
                self.sub_list.append(
                    self.create_subscription(Image, topic, self.make_image_callback(topic), 10)
                )
                self.image_data[topic] = None

        self.create_timer(0.05, self.visualize_callback)

    def make_image_callback(self, topic):
        def callback(msg):
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.image_data[topic] = img
        return callback

    def lidar_callback(self, msg):
        points = np.array([
            [p[0], p[1]] for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        ])
        self.lidar_data = points

    def visualize_callback(self):
        window_size = (640, 360)

        for topic, img in self.image_data.items():
            if img is not None:
                resized = cv2.resize(img, window_size)
                cv2.imshow(topic, resized)

        if self.lidar_data is not None:
            canvas = np.zeros((window_size[1], window_size[0], 3), dtype=np.uint8)

            # Normalize and scale points into canvas space
            scaled_pts = self.lidar_data[:, :2] * 10  # scale for visibility
            scaled_pts[:, 0] += window_size[0] // 2
            scaled_pts[:, 1] += window_size[1] // 2

            for x, y in scaled_pts.astype(int):
                if 0 <= x < window_size[0] and 0 <= y < window_size[1]:
                    canvas[y, x] = (255, 255, 255)

            cv2.imshow("LiDAR", canvas)

        cv2.waitKey(1)

def visualize_sensors_from_config():
    rclpy.init()
    node = SensorVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    visualize_sensors_from_config()
