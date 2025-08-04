import time
import rclpy
import carla
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


class CarlaRosPublisher(Node):
    """ROS node for publishing CARLA LiDAR data."""

    def __init__(self):
        super().__init__("ros_publisher")
        self.tf = TransformBroadcaster(self)

        self.lidar_publisher = self.create_publisher(
            PointCloud2, "carla/lidar/pointcloud", 10
        )

        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        self.sensor = None
        self.attach_lidar_to_cav()

    def attach_lidar_to_cav(self):
        """Find CAV and attach a LiDAR sensor to it."""
        cav = None
        while cav is None:
            actor_list = self.world.get_actors().filter("vehicle.*")
            for actor in actor_list:
                if actor.attributes.get("role_name") == "CAV":
                    cav = actor
                    break
            if cav is None:
                print("CAV not found. Waiting for CAV to spawn...")
                time.sleep(1.0)

        print("Found CAV, attaching LiDAR sensor...")

        bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
        lidar_settings = {
            "channels": "64",
            "range": "40",
            "points_per_second": "480000",
            "rotation_frequency": "15",
            "upper_fov": "0.0",
            "lower_fov": "-23.8",
            "dropoff_general_rate": "0.0",
            "noise_stddev": "0.0",
            "horizontal_fov": "360.0",
        }
        for key, value in lidar_settings.items():
            bp.set_attribute(key, value)

        lidar_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.4))
        self.sensor = self.world.spawn_actor(bp, lidar_transform, attach_to=cav)
        self.sensor.listen(self.lidar_callback)

    def lidar_callback(self, image):
        """Handle LiDAR data and publish as ROS PointCloud2."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "carla_lidar"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Make the array writable with `.copy()`
        lidar_data = np.frombuffer(image.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        lidar_data[:, 1] *= -1  # Convert CARLA coordinate system to ROS

        pointcloud = self.create_cloud(header, fields, lidar_data)
        self.lidar_publisher.publish(pointcloud)

        self.publish_static_transform()

    def create_cloud(self, header, fields, points):
        data = np.asarray(points, dtype=np.float32).tobytes()
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

    def publish_static_transform(self):
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


def main():
    rclpy.init(args=None)
    node = CarlaRosPublisher()
    try:
        rclpy.spin(node)
    finally:
        if node.sensor:
            node.sensor.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()