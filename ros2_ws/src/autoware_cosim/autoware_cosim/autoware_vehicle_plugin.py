import time
import math
import rclpy
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from autoware_auto_perception_msgs.msg import (
    DetectedObjects,
    DetectedObject,
    ObjectClassification,
    Shape,
)

from terasim_cosim.constants import *
from terasim_cosim.redis_msgs import Vehicle, VehicleDict
from terasim_cosim.redis_client_wrapper import create_redis_client

from math import atan2, cos, sin, pi


class AutowareVehiclePlugin(Node):

    def __init__(self):
        super().__init__("autoware_vehicle_plugin")

        self.declare_parameter("control_cav", True)
        self.declare_parameter(
            "cosim_controlled_vehicle_keys", [TERASIM_COSIM_VEHICLE_INFO]
        )

        self.control_cav = (
            self.get_parameter("control_cav").get_parameter_value().bool_value
        )
        self.cosim_controlled_vehicle_keys = (
            self.get_parameter("cosim_controlled_vehicle_keys")
            .get_parameter_value()
            .string_array_value
        )

        # autoware cav localization and display
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            10,
        )
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped,
            "/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )
        self.pub_odom = self.create_publisher(
            Odometry, "/localization/pose_twist_fusion_filter/kinematic_state", 10
        )

        # autoware perception
        self.pub_detected_objects = self.create_publisher(
            DetectedObjects, "/perception/object_recognition/detection/objects", 10
        )

        self.sub_ego_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )

        self.tf = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.on_timer)

        self.saved_odom_msg = Odometry()

        self.UTM_offset = [-277497.10, -4686518.71, 0.0]

        self.on_start()

        print("Starting autoware vehicle interface with redis...")

    def on_start(self):
        key_value_config = {
            CAV_COSIM_VEHICLE_INFO: VehicleDict,
        }
        for key in self.cosim_controlled_vehicle_keys:
            key_value_config[key] = VehicleDict

        self.redis_client = create_redis_client(key_value_config=key_value_config)

    def on_timer(self):
        if self.control_cav:
            self.sync_autoware_cav_to_cosim()
        else:
            self.sync_cosim_cav_to_autoware()

        self.sync_cosim_vehicle_to_autoware()

    def sync_autoware_cav_to_cosim(self):
        x = self.saved_odom_msg.pose.pose.position.x - self.UTM_offset[0]
        y = self.saved_odom_msg.pose.pose.position.y - self.UTM_offset[1]
        speed_long = self.saved_odom_msg.twist.twist.linear.x

        qx = self.saved_odom_msg.pose.pose.orientation.x
        qy = self.saved_odom_msg.pose.pose.orientation.y
        qz = self.saved_odom_msg.pose.pose.orientation.z
        qw = self.saved_odom_msg.pose.pose.orientation.w

        orientation = self.get_orientation_from_quaternion(qx, qy, qz, qw)
        x, y = self.autoware_coordinate_to_center_coordinate(x, y, orientation)

        cav_cosim_vehicle_info = VehicleDict()
        cav_cosim_vehicle_info.header.timestamp = time.time()

        info = Vehicle()
        info.x = x
        info.y = y
        info.z = 275.0
        info.length = 5.0
        info.width = 1.8
        info.height = 1.5
        info.orientation = orientation
        info.speed_long = speed_long

        cav_cosim_vehicle_info.data["CAV"] = info

        self.redis_client.set(CAV_COSIM_VEHICLE_INFO, cav_cosim_vehicle_info)

    def sync_cosim_cav_to_autoware(self):
        try:
            cav_cosim_vehicle_info = self.redis_client.get(CAV_COSIM_VEHICLE_INFO)
        except:
            print("cav_cosim_vehicle_info not available. Exiting...")
            return

        if cav_cosim_vehicle_info:
            data = cav_cosim_vehicle_info.data
            cav_info = data["CAV"]

            cav_x = cav_info.x + self.UTM_offset[0]
            cav_y = cav_info.y + self.UTM_offset[1]
            cav_z = cav_info.z

            cav_orientation = cav_info.orientation

            cav_x, cav_y = self.center_coordinate_to_autoware_coordinate(
                cav_x, cav_x, cav_orientation
            )

            # Modify position based on some offsets and set to pose message
            pose_with_cov_msg = PoseWithCovarianceStamped()
            pose_with_cov_msg.pose.pose.position.x = cav_x
            pose_with_cov_msg.pose.pose.position.y = cav_y

            # Extract and process orientation data
            qx, qy, qz, qw = self.get_quaternion_from_orientation(cav_orientation)

            pose_with_cov_msg.pose.pose.orientation.x = qx
            pose_with_cov_msg.pose.pose.orientation.y = qy
            pose_with_cov_msg.pose.pose.orientation.z = qz
            pose_with_cov_msg.pose.pose.orientation.w = qw

            # Set linear velocities from odom to twist message
            twist_with_cov_msg = TwistWithCovarianceStamped()

            twist_with_cov_msg.twist.twist.linear.x = cav_info.speed_long
            twist_with_cov_msg.twist.twist.linear.y = 0.0
            twist_with_cov_msg.twist.twist.linear.z = 0.0

            pose_with_cov_msg.pose.covariance = np.eye(6).flatten().tolist()
            twist_with_cov_msg.twist.covariance = np.eye(6).flatten().tolist()

            odom_msg = Odometry()
            odom_msg.pose.pose = pose_with_cov_msg.pose.pose
            odom_msg.twist.twist = twist_with_cov_msg.twist.twist

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"

            # Assign the same header to all messages
            pose_with_cov_msg.header = header
            twist_with_cov_msg.header = header
            odom_msg.header = header

            # Publish messages
            self.pub_pose.publish(pose_with_cov_msg)
            self.pub_twist.publish(twist_with_cov_msg)
            self.pub_odom.publish(odom_msg)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"

            t.transform.translation.x = cav_x
            t.transform.translation.y = cav_y
            t.transform.translation.z = cav_z

            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf.sendTransform(t)

    def sync_cosim_vehicle_to_autoware(self):
        self.detected_objects_msg = DetectedObjects()

        for key in self.cosim_controlled_vehicle_keys:
            try:
                cosim_controlled_vehicle_info = self.redis_client.get(key)
            except:
                print(key + " not found available. Exiting...")
                continue

            if cosim_controlled_vehicle_info:
                data = cosim_controlled_vehicle_info.data

                for vehID in data:
                    if vehID != "CAV":
                        self.update_perception_in_autoware(vehID, data[vehID])

        self.detected_objects_msg.header.stamp = self.get_clock().now().to_msg()
        self.detected_objects_msg.header.frame_id = "map"
        self.pub_detected_objects.publish(self.detected_objects_msg)

    def get_pose_with_variance(self, bv_info):
        bv_pose_with_covariance = PoseWithCovariance()

        x = bv_info.x + self.UTM_offset[0]
        y = bv_info.y + self.UTM_offset[1]
        orientation = bv_info.orientation

        bv_pose_with_covariance.pose.position.x = x
        bv_pose_with_covariance.pose.position.y = y
        bv_pose_with_covariance.pose.orientation.w = cos(orientation / 2)
        bv_pose_with_covariance.pose.orientation.x = 0.0
        bv_pose_with_covariance.pose.orientation.y = 0.0
        bv_pose_with_covariance.pose.orientation.z = sin(orientation / 2)

        return bv_pose_with_covariance

    def get_twist_with_variance(self, bv_info):
        bv_twist_with_covariance = TwistWithCovariance()
        bv_twist_with_covariance.twist.linear.x = bv_info.speed_long
        return bv_twist_with_covariance

    def get_shape(self, bv_info):
        bv_shape = Shape()
        bv_shape.type = Shape.BOUNDING_BOX
        bv_shape.dimensions.x = bv_info.length
        bv_shape.dimensions.y = bv_info.width
        bv_shape.dimensions.z = bv_info.height
        return bv_shape

    def get_classification(self, bv_key):
        bv_classification = ObjectClassification()

        if "POV" in bv_key:
            bv_classification.label = ObjectClassification.CAR
        elif "BV" in bv_key:
            bv_classification.label = ObjectClassification.CAR
        elif "CV" in bv_key:
            bv_classification.label = ObjectClassification.CAR
        elif "CARLA" in bv_key:
            bv_classification.label = ObjectClassification.CAR
        elif "VRU" in bv_key:
            bv_classification.label = ObjectClassification.PEDESTRIAN
        else:
            bv_classification.label = ObjectClassification.UNKNOWN

        bv_classification.probability = 1.0
        return bv_classification

    def update_perception_in_autoware(self, bv_key, bv_info):
        detected_object = DetectedObject()
        detected_object.existence_probability = 1.0
        detected_object.classification.append(self.get_classification(bv_key))
        detected_object.kinematics.pose_with_covariance = self.get_pose_with_variance(
            bv_info
        )
        detected_object.kinematics.has_position_covariance = False
        detected_object.kinematics.orientation_availability = 0
        detected_object.kinematics.twist_with_covariance = self.get_twist_with_variance(
            bv_info
        )
        detected_object.kinematics.has_twist = False
        detected_object.kinematics.has_twist_covariance = False
        detected_object.shape = self.get_shape(bv_info)

        self.detected_objects_msg.objects.append(detected_object)

    def center_coordinate_to_autoware_coordinate(
        self, x, y, heading, rear_shaft_to_center=1.5
    ):
        """
        Convert the center coordinate to the Autoware coordinate. the input will be a list of {x, y, heading, vx}.
        """
        x = x - math.cos(heading) * rear_shaft_to_center
        y = y - math.sin(heading) * rear_shaft_to_center
        return x, y

    def autoware_coordinate_to_center_coordinate(
        self, x, y, heading, rear_shaft_to_center=1.5
    ):
        """
        Convert the Autoware coordinate to the center coordinate. the input will be a list of {x, y, heading, vx}.
        """
        x = x + math.cos(heading) * rear_shaft_to_center
        y = y + math.sin(heading) * rear_shaft_to_center
        return x, y

    def get_orientation_from_quaternion(self, qx, qy, qz, qw):
        orientation = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        while orientation > pi:
            orientation -= 2.0 * pi
        while orientation < -pi:
            orientation += 2.0 * pi

        return orientation

    def get_quaternion_from_orientation(self, orientation):
        w = cos(orientation / 2.0)
        x = 0.0
        y = 0.0
        z = sin(orientation / 2.0)

        return x, y, z, w

    def odom_callback(self, msg):
        self.saved_odom_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = AutowareVehiclePlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
