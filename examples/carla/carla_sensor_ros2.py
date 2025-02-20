import cv2
import time
import rclpy
import carla
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


# CARLA sensor manager class
class SensorManager:
    """Manages CARLA sensors and publishes data to ROS."""

    def __init__(
        self, world, sensor_type, transform, attached, sensor_options, ros_publisher
    ):
        self.world = world
        self.sensor_options = sensor_options
        self.ros_publisher = ros_publisher
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == "Spectator":
            spectator = self.world.get_spectator()

            camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")

            for key, value in sensor_options.items():
                camera_bp.set_attribute(key, value)

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=spectator)
            camera.listen(self.spectator_callback)

            return camera

        elif sensor_type == "RGBCamera":
            camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")

            for key, value in sensor_options.items():
                camera_bp.set_attribute(key, value)

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.camera_callback)

            return camera

        elif sensor_type == "Segmentation":
            segmentation_bp = self.world.get_blueprint_library().find(
                "sensor.camera.semantic_segmentation"
            )

            for key, value in sensor_options.items():
                segmentation_bp.set_attribute(key, value)

            segmentation = self.world.spawn_actor(segmentation_bp, transform)
            segmentation.listen(self.segmentation_callback)
            return segmentation

        elif sensor_type == "DepthCamera":
            depth_bp = self.world.get_blueprint_library().find("sensor.camera.depth")

            for key, value in sensor_options.items():
                depth_bp.set_attribute(key, value)

            depth = self.world.spawn_actor(depth_bp, transform, attach_to=attached)
            depth.listen(self.depth_callback)
            return depth

        elif sensor_type == "LiDAR":
            lidar_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")

            for key, value in sensor_options.items():
                lidar_bp.set_attribute(key, value)

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
            lidar.listen(self.lidar_callback)
            return lidar

        return None

    def spectator_callback(self, image):
        self.ros_publisher.save_spectator(image)

    def camera_callback(self, image):
        self.ros_publisher.save_camera(image)

    def lidar_callback(self, image):
        self.ros_publisher.save_lidar(image)

    def segmentation_callback(self, image):
        self.ros_publisher.save_segmentation(image)

    def depth_callback(self, image):
        self.ros_publisher.save_depth(image)

    def destroy(self):
        """Clean up the sensor."""
        self.sensor.destroy()


# ROS publisher node for CARLA sensor data
class CarlaRosPublisher(Node):
    """ROS node for publishing CARLA sensor data."""

    def __init__(self):
        super().__init__("ros_publisher")
        self.tf = TransformBroadcaster(self)

        self.lidar_publisher = self.create_publisher(
            PointCloud2, "/sensing/lidar/concatenated/pointcloud", 10
        )

        self.spectator_publisher = self.create_publisher(
            CompressedImage, "/camera/spectator/compressed", 10
        )

        self.camera_publisher = self.create_publisher(
            CompressedImage, "/camera/image_color/compressed", 10
        )

        # Raw segmentation image
        self.segmentation_publisher = self.create_publisher(
            CompressedImage, "/camera/segmentation/compressed", 10
        )

        # Enhanced segmentation image for object visualization
        self.segmentation_visualization_publisher = self.create_publisher(
            CompressedImage, "/camera/segmentation_visualization/compressed", 10
        )

        # Raw depth image
        self.depth_publisher = self.create_publisher(
            CompressedImage, "/camera/depth/compressed", 10
        )

        # Enhanced depth image for depth visualization
        self.depth_visualization_publisher = self.create_publisher(
            CompressedImage, "/camera/depth_visualization/compressed", 10
        )

        # Timer for publishing saved images
        self.create_timer(0.05, self.timer_callback)

        # Attach sensors to the CAV
        self.attach_carla_sensor()

    def attach_carla_sensor(self):
        """
        Find the vehicle with the role_name "CAV" and attach sensors to it.
        A list of example sensor setup are provided below (LiDAR & Camera) with adjuable parameters.
        The sensor output will be published to ROS topics.
        """

        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(2.0)
        world = client.get_world()

        CAV = None

        self.saved_spectator_image = None
        self.saved_lidar_image = None
        self.saved_camera_image = None
        self.saved_depth_image = None
        self.saved_segmentation_image = None

        while CAV is None:
            actor_list = world.get_actors().filter("vehicle.*")
            for x in actor_list:
                if x.attributes.get("role_name") == "CAV":
                    CAV = x
            print("CAV not found. Waiting for CAV to spawn...")
            time.sleep(0.1)

        print("Found CAV, attach sensors...")

        # # Creates a LiDAR sensor
        # SensorManager(
        #     world,
        #     "LiDAR",
        #     carla.Transform(carla.Location(x=0.0, y=0.0, z=2.4)),
        #     CAV,
        #     {
        #         "channels": "64",
        #         "range": "30",
        #         "points_per_second": "320000",
        #         "rotation_frequency": "25",
        #         "upper_fov": "0.0",
        #         "lower_fov": "-26.8",
        #         "dropoff_general_rate": "0.0",
        #         "noise_stddev": "0.0",
        #         "horizontal_fov": "360.0"
        #     },
        #     ros_publisher=self,
        # )

        # # Creates a bird-eye view camera
        # SensorManager(
        #     world,
        #     "Spectator",
        #     carla.Transform(carla.Location(), carla.Rotation()),
        #     CAV,
        #     {
        #         "image_size_x": "1920",
        #         "image_size_y": "1080",
        #     },
        #     ros_publisher=self,
        # )

        # # Creates a RGB camera from the front of the AV
        # SensorManager(
        #     world,
        #     "RGBCamera",
        #     carla.Transform(carla.Location(x=0.5, y=0.2, z=1.4), carla.Rotation()),
        #     CAV,
        #     {
        #         "image_size_x": "1920",
        #         "image_size_y": "1080",
        #     },
        #     ros_publisher=self,
        # )

        # Creates a RGB camera to follow the AV from the back
        SensorManager(
            world,
            "RGBCamera",
            carla.Transform(
                carla.Location(x=-11.0, y=0.0, z=7.0),
                carla.Rotation(roll=0.0, pitch=-25.0, yaw=0.0),
            ),
            CAV,
            {
                "image_size_x": "1920",
                "image_size_y": "1080",
            },
            ros_publisher=self,
        )

        # # Creates a segmentation camera from the front of the AV
        # SensorManager(
        #     world,
        #     "Segmentation",
        #     carla.Transform(carla.Location(x=0.5, y=0.2, z=1.4), carla.Rotation()),
        #     CAV,
        #     {
        #         "image_size_x": "1920",
        #         "image_size_y": "1080",
        #     },
        #     ros_publisher=self,
        # )

        # # Creates a depth camera from the front of the AV
        # SensorManager(
        #     world,
        #     "DepthCamera",
        #     carla.Transform(carla.Location(x=0.5, y=0.2, z=1.4), carla.Rotation()),
        #     CAV,
        #     {
        #         "image_size_x": "1920",
        #         "image_size_y": "1080",
        #     },
        #     ros_publisher=self,
        # )

    def timer_callback(self):
        """Timer callback for publishing saved LiDAR images."""
        if self.saved_lidar_image is not None:
            self.publish_lidar(self.saved_lidar_image)
        if self.saved_spectator_image is not None:
            self.publish_spectator(self.saved_spectator_image)
        if self.saved_camera_image is not None:
            self.publish_camera(self.saved_camera_image)
        if self.saved_segmentation_image is not None:
            self.publish_segmentation(self.saved_segmentation_image)
            self.publish_segmentation_visualization(self.saved_segmentation_image)
        if self.saved_depth_image is not None:
            self.publish_depth(self.saved_depth_image)
            self.publish_depth_visualization(self.saved_depth_image)

    def save_lidar(self, image):
        """Save lidar image for later publishing."""
        self.saved_lidar_image = image

    def save_spectator(self, image):
        self.saved_spectator_image = image

    def save_camera(self, image):
        self.saved_camera_image = image

    def save_segmentation(self, image):
        self.saved_segmentation_image = image

    def save_depth(self, image):
        self.saved_depth_image = image

    def publish_spectator(self, image):
        # Ensure the image data is converted correctly
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4)
        )  # BGRA format from Carla
        img_bgr = array[:, :, :3]  # Remove alpha channel

        # Convert to JPEG
        _, img_compressed = cv2.imencode(".jpg", img_bgr)

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(img_compressed).tobytes()

        # Publish the message
        self.spectator_publisher.publish(msg)

    def publish_camera(self, image):
        # Ensure the image data is converted correctly
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4)
        )  # BGRA format from Carla
        img_bgr = array[:, :, :3]  # Remove alpha channel

        # Convert to JPEG
        _, img_compressed = cv2.imencode(".jpg", img_bgr)

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(img_compressed).tobytes()

        # Publish the message
        self.camera_publisher.publish(msg)

    def publish_segmentation(self, image):
        # Ensure the image data is converted correctly
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4)
        )  # BGRA format from Carla
        img_bgr = array[:, :, :3]  # Remove alpha channel

        # Convert to JPEG
        _, img_compressed = cv2.imencode(".jpg", img_bgr)

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(img_compressed).tobytes()

        # Publish the message
        self.segmentation_publisher.publish(msg)

    def publish_segmentation_visualization(self, image):
        image.convert(carla.ColorConverter.CityScapesPalette)

        # Ensure the image data is converted correctly
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4)
        )  # BGRA format from Carla
        img_bgr = array[:, :, :3]  # Remove alpha channel

        # Convert to JPEG
        _, img_compressed = cv2.imencode(".jpg", img_bgr)

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(img_compressed).tobytes()

        # Publish the message
        self.segmentation_visualization_publisher.publish(msg)

    def publish_depth(self, image):
        # Ensure the image data is converted correctly
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4)
        )  # BGRA format from Carla
        img_bgr = array[:, :, :3]  # Remove alpha channel

        # Convert to JPEG
        _, img_compressed = cv2.imencode(".jpg", img_bgr)

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(img_compressed).tobytes()

        # Publish the message
        self.depth_publisher.publish(msg)

    def publish_depth_visualization(self, image):
        # Ensure the image data is converted correctly
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4)
        )  # BGRA format from Carla

        # Extract B, G, R channels
        B = array[:, :, 0].astype(np.float32)
        G = array[:, :, 1].astype(np.float32)
        R = array[:, :, 2].astype(np.float32)

        # Apply depth conversion algorithm
        normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        depth_in_meters = 1000 * normalized

        # Set minimum and maximum depth values for clipping
        min_depth = 0.1
        max_depth = 255.0

        # Clip depth values to enhance contrast
        depth_clipped = np.clip(depth_in_meters, min_depth, max_depth)

        # Apply logarithmic scaling
        depth_log = np.log1p(depth_clipped)

        # Normalize to the 8-bit range
        depth_normalized = depth_log / np.max(depth_log)
        depth_image_8bit = np.uint8(depth_normalized * 255)

        # Convert the grayscale image to a JPEG format
        _, img_compressed = cv2.imencode(".jpg", depth_image_8bit)

        # Create a CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(img_compressed).tobytes()

        # Publish the grayscale depth image message
        self.depth_visualization_publisher.publish(msg)

    def publish_lidar(self, image):
        """Publish lidar data as a ROS point cloud."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "carla_lidar"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        lidar_data = np.fromstring(bytes(image.raw_data), dtype=np.float32)
        lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))

        lidar_data[:, 1] *= -1  # Adjust coordinate system

        point_cloud_msg = self.create_cloud(header, fields, lidar_data)
        self.lidar_publisher.publish(point_cloud_msg)

        # Update transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "carla_lidar"
        t.transform.translation.x = 2.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 2.4
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf.sendTransform(t)

    def create_cloud(self, header, fields, points):
        """Create a ROS point cloud from lidar data."""
        data = np.array(points, dtype=np.float32).tobytes()
        return PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * len(points),
            data=data,
        )


def main():
    rclpy.init(args=None)
    node = CarlaRosPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
