import time
import yaml
import rclpy
import carla
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from functools import partial


class SensorManager:
    def __init__(self, world, sensor_type, transform, attached, sensor_options, ros_publisher, topic):
        self.world = world
        self.sensor_options = sensor_options
        self.ros_publisher = ros_publisher
        self.topic = topic
        self.sensor = self.init_sensor(sensor_type, transform, attached)

    def init_sensor(self, sensor_type, transform, attached):
        blueprint_library = self.world.get_blueprint_library()

        def create_sensor(bp_name, callback):
            bp = blueprint_library.find(bp_name)
            for key, value in self.sensor_options.items():
                bp.set_attribute(key, value)
            actor = self.world.spawn_actor(bp, transform, attach_to=attached)
            actor.listen(callback)
            return actor

        if sensor_type == "RGBCamera":
            return create_sensor("sensor.camera.rgb", partial(self.ros_publisher.camera_callback, topic=self.topic))
        elif sensor_type == "SegCamera":
            return create_sensor("sensor.camera.semantic_segmentation", partial(self.ros_publisher.segmentation_callback, topic=self.topic))
        elif sensor_type == "DepthCamera":
            return create_sensor("sensor.camera.depth", partial(self.ros_publisher.depth_callback, topic=self.topic))
        elif sensor_type == "LiDAR":
            return create_sensor("sensor.lidar.ray_cast", self.ros_publisher.lidar_callback)

        return None

    def destroy(self):
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()


class CarlaRosPublisher(Node):
    def __init__(self):
        super().__init__("carla_sensor_ros2")
        self.tf = TransformBroadcaster(self)
        self.bridge = CvBridge()

        self.saved_images = {}
        self.lidar_data = None
        self.lidar_topic = None  # <-- store LiDAR topic

        self.create_timer(0.05, self.timer_callback)
        self.load_and_attach_sensors()

    def load_and_attach_sensors(self):
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(2.0)
        world = client.get_world()

        with open("sensor_config.yaml", "r") as f:
            config = yaml.safe_load(f)

        cav = None
        while cav is None:
            for actor in world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") == "CAV":
                    cav = actor
            print("CAV not found. Waiting...")
            time.sleep(0.1)
        print("Found CAV, attaching sensors...")

        self.pub_dict = {}
        self.sensor_instances = []

        for sensor_cfg in config["sensors"]:
            s_type = sensor_cfg["type"]
            topic = sensor_cfg["topic"]
            transform = carla.Transform(
                carla.Location(**sensor_cfg["transform"]["location"]),
                carla.Rotation(**sensor_cfg["transform"]["rotation"])
            )
            options = sensor_cfg.get("options", {})
            pub_type = PointCloud2 if s_type == "LiDAR" else Image
            self.pub_dict[topic] = self.create_publisher(pub_type, topic, 10)

            if s_type == "LiDAR":
                self.lidar_topic = topic  # <-- track LiDAR topic

            sensor = SensorManager(world, s_type, transform, cav, options, self, topic)
            self.sensor_instances.append(sensor)

    def destroy_all_sensors(self):
        for sensor in getattr(self, "sensor_instances", []):
            try:
                sensor.destroy()
                self.get_logger().info(f"Destroyed sensor on topic {sensor.topic}")
            except Exception as e:
                self.get_logger().warn(f"Failed to destroy sensor: {e}")

    def camera_callback(self, image, topic):
        array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        img_bgr = array[:, :, :3]
        msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.saved_images[topic] = msg

    def segmentation_callback(self, image, topic):
        image.convert(carla.ColorConverter.CityScapesPalette)
        array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        img_bgr = array[:, :, :3]
        msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.saved_images[topic] = msg

    def depth_callback(self, image, topic):
        array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        B, G, R = array[:, :, 0].astype(np.float32), array[:, :, 1].astype(np.float32), array[:, :, 2].astype(np.float32)
        normalized = (R + G * 256 + B * 256 * 256) / (256**3 - 1)
        depth = 1000 * normalized
        depth_clipped = np.clip(depth, 0.1, 255.0)
        depth_log = np.log1p(depth_clipped)
        depth_norm = depth_log / np.max(depth_log)
        depth_img = (depth_norm * 255).astype(np.uint8)
        msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="mono8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.saved_images[topic] = msg

    def lidar_callback(self, image):
        self.lidar_data = image

    def timer_callback(self):
        for topic, msg in list(self.saved_images.items()):  # make a copy to prevent mutation during iteration
            if topic in self.pub_dict:
                self.pub_dict[topic].publish(msg)

        if self.lidar_data:
            self.publish_lidar(self.lidar_data)

    def publish_lidar(self, image):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "carla_lidar"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        lidar_data = np.frombuffer(image.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        lidar_data[:, 1] *= -1  # Convert CARLA coordinate system to ROS

        pc2 = PointCloud2(
            header=header,
            height=1,
            width=len(lidar_data),
            fields=fields,
            is_bigendian=False,
            is_dense=False,
            point_step=16,
            row_step=16 * len(lidar_data),
            data=lidar_data.tobytes()
        )

        if self.lidar_topic in self.pub_dict:
            self.pub_dict[self.lidar_topic].publish(pc2)
        else:
            self.get_logger().warn(f"LiDAR topic '{self.lidar_topic}' not found in pub_dict")

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "carla_lidar"
        t.transform.translation.x = 2.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 2.4
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)


def cleanup_node(node):
    print("Shutting down... destroying sensors")
    if hasattr(node, "destroy_all_sensors"):
        node.destroy_all_sensors()
    node.destroy_node()
    rclpy.shutdown()
    time.sleep(0.1)  # Give native threads time to finalize


def main():
    rclpy.init()
    node = CarlaRosPublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        cleanup_node(node)


if __name__ == "__main__":
    main()
