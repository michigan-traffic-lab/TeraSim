import numpy as np
import torch
from torchvision.ops import nms

import rclpy
from rclpy.node import Node
import ros2_numpy
import tf2_ros
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from autoware_auto_perception_msgs.msg import DetectedObject, DetectedObjects, ObjectClassification, Shape

# Register PoseStamped for TF2
import tf2_geometry_msgs.tf2_geometry_msgs

import threading
import time

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from pcdet.datasets.dataset import DatasetTemplate


CONFIDENCE_THRESHOLD = 0.75
CFG_PATH = "voxel_rcnn_car.yaml"
MODEL_PATH = "voxel_rcnn_car_84.54.pth"


class DummyDataset(DatasetTemplate):
    def __init__(self, cfg, class_names):
        super().__init__(dataset_cfg=cfg.DATA_CONFIG, class_names=class_names, training=False)

    def __len__(self):
        return 0

    def __getitem__(self, index):
        raise IndexError


class PcdetNode(Node):
    def __init__(self):
        super().__init__('pc_dection_node')

        self.latest_msg = None
        self.lock = threading.Lock()

        self.lidar_sub = self.create_subscription(PointCloud2, 'carla/lidar/pointcloud', self.call_back, 10)
        self.detection_pub = self.create_publisher(DetectedObjects, '/perception/object_recognition/detection/objects', 10)

        cfg_from_yaml_file(CFG_PATH, cfg)
        self.dummmydataset = DummyDataset(cfg=cfg, class_names=cfg.CLASS_NAMES)
        self.model = build_network(cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=self.dummmydataset)

        logger_ = common_utils.create_logger("ros-pcdet", log_level='ERROR')
        self.model.load_params_from_file(MODEL_PATH, logger=logger_, to_cpu=False)
        self.model.to('cuda').eval()

        self.get_logger().info("Pcdet node initialized")

        self.tf2_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf2_buf, self, spin_thread=True)

        threading.Thread(target=self.process_loop, daemon=True).start()

    def call_back(self, msg: PointCloud2):
        with self.lock:
            self.latest_msg = msg  # Always overwrite with the latest message

    def process_loop(self):
        while rclpy.ok():
            time.sleep(0.01)  # small sleep to reduce CPU usage
            with self.lock:
                msg = self.latest_msg
                self.latest_msg = None  # Mark as consumed
            if msg:
                self._process_lidar(msg)

    def _process_lidar(self, msg: PointCloud2):
        cloud_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        tensor_points = self.convert_cloud(cloud_array)

        pts_rotated = tensor_points.clone()
        pts_rotated[:, :2].mul_(-1)

        batch_boxes, batch_scores, _ = self.run_inference_batch([tensor_points, pts_rotated])
        if batch_boxes is None:
            return

        boxes1, boxes2 = batch_boxes
        scores1, scores2 = batch_scores

        boxes2[:, :2] *= -1
        boxes2[:, 6] = (boxes2[:, 6] + torch.pi + torch.pi) % (2 * torch.pi) - torch.pi

        boxes = torch.cat((boxes1, boxes2), dim=0)
        scores = torch.cat((scores1, scores2), dim=0)

        high_conf_mask = scores >= CONFIDENCE_THRESHOLD
        boxes = boxes[high_conf_mask]
        scores = scores[high_conf_mask]

        if len(scores) == 0:
            return

        order = scores.argsort(descending=True)
        x, y, w, l = boxes[:, 0], boxes[:, 1], boxes[:, 3], boxes[:, 4]
        boxes_aabb = torch.stack([x - w / 2, y - l / 2, x + w / 2, y + l / 2], 1)
        keep = nms(boxes_aabb, scores=scores[order], iou_threshold=0.5)

        boxes = boxes[keep]
        scores = scores[keep]

        obj_msg = DetectedObjects()
        obj_msg.header.frame_id = "map"
        obj_msg.header.stamp = self.get_clock().now().to_msg()

        for box, score in zip(boxes, scores):
            pose_map = self.pose_transform(float(box[0]), float(box[1]), 0.0, float(box[6]))
            if pose_map is None:
                continue

            obj = DetectedObject()
            obj.existence_probability = float(score.item())
            obj.classification.append(ObjectClassification(label=ObjectClassification.CAR, probability=1.0))
            obj.kinematics.pose_with_covariance.pose.position.x = pose_map.pose.position.x
            obj.kinematics.pose_with_covariance.pose.position.y = pose_map.pose.position.y
            obj.kinematics.pose_with_covariance.pose.position.z = 0.8
            obj.kinematics.pose_with_covariance.pose.orientation = pose_map.pose.orientation
            obj.kinematics.has_position_covariance = False
            obj.kinematics.orientation_availability = 0
            obj.kinematics.has_twist_covariance = False
            obj.shape = self.get_shape()
            obj_msg.objects.append(obj)

        self.detection_pub.publish(obj_msg)

    def run_inference_batch(self, points_list):
        input_dicts = [{'points': p} for p in points_list]
        with torch.no_grad():
            batch = [self.dummmydataset.prepare_data(data_dict=d) for d in input_dicts]
            batch = self.dummmydataset.collate_batch(batch)
            load_data_to_gpu(batch)
            pred_dicts, _ = self.model(batch)
            torch.cuda.synchronize()

        all_boxes = [pred['pred_boxes'] for pred in pred_dicts]
        all_scores = [pred['pred_scores'] for pred in pred_dicts]
        all_labels = [pred['pred_labels'] for pred in pred_dicts]
        return all_boxes, all_scores, all_labels

    def convert_cloud(self, cloud_dict, remove_nan=True, dtype=np.float32):
        xyz = cloud_dict['xyz']
        if remove_nan:
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
        pts = np.zeros((xyz.shape[0], 4), dtype=dtype)
        pts[:, :3] = xyz
        return torch.from_numpy(pts).float()

    def pose_transform(self, x: float, y: float, z: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "carla_lidar"
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        quat = R.from_euler("z", yaw).as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat
        try:
            return self.tf2_buf.transform(pose, 'map', timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            self.get_logger().warn("TF not ready, skip this frame")
            return None

    def get_shape(self) -> Shape:
        s = Shape()
        s.type = Shape.BOUNDING_BOX
        s.dimensions.x = 5.0
        s.dimensions.y = 1.85
        s.dimensions.z = 1.5
        return s


def main(args=None):
    rclpy.init(args=args)
    node = PcdetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
