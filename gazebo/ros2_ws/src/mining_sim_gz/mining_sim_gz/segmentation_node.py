"""Label-free segmentation of the accumulated LiDAR map (floor / ceiling / wall / object from geometry only).

Accumulates map points (voxel-deduplicated, pose from `pose_source`), and on a Trigger call to /segment_map (or every `period` seconds)
segments them, publishes a coloured cloud on /semantic/points and, when ground-truth files are given, scores the result against the
exported cave mesh + formations (those labels are NEVER used to segment).
"""
import json
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .cloud_utils import level_points, make_cloud, pointcloud2_to_xyz
from .ground_truth import CaveGroundTruth
from .segmenter import CLASSES, evaluate_segmentation, segment_points
from .se2 import yaw_from_quat

COLORS = np.array([[217, 164, 65], [143, 123, 255], [63, 140, 255], [255, 106, 0]], dtype=np.uint8)   # floor, ceiling, wall, object


class SegmentationNode(Node):
    def __init__(self):
        super().__init__("segmentation")
        p = self.declare_parameter
        self.pose_source = p("pose_source", "ground_truth").value
        self.sensor_h = p("sensor_height", 0.972).value
        self.log_dir = p("log_dir", "").value
        mesh, info = p("gt_mesh", "").value, p("info_json", "").value
        self.gt = CaveGroundTruth(mesh, info) if mesh and os.path.exists(mesh) else None
        self.pose = self.imu_q = None
        self.seen, self.pts, self.views = set(), [], []
        self.last_view = None
        if self.pose_source == "slam":
            self.create_subscription(PoseStamped, "/slam/pose", lambda m: self._set(m.pose), 10)
        else:
            self.create_subscription(Odometry, "/ground_truth/odom", lambda m: self._set(m.pose.pose), 20)
        self.create_subscription(Imu, "/imu", lambda m: setattr(self, "imu_q", (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)), 20)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, "/semantic/points", 1)
        self.result_pub = self.create_publisher(String, "/semantic/result", 1)
        self.create_service(Trigger, "/segment_map", self.on_trigger)
        period = p("period", 0.0).value
        if period > 0:
            self.create_timer(period, lambda: self.run())
        self.get_logger().info("segmentation ready: call  ros2 service call /segment_map std_srvs/srv/Trigger")

    def _set(self, pose):
        q = pose.orientation
        self.pose = (pose.position.x, pose.position.y, pose.position.z, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_cloud(self, msg):
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        lvl = level_points(pointcloud2_to_xyz(msg, max_range=25.0), self.imu_q)
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.stack([c * lvl[:, 0] - s * lvl[:, 1] + x, s * lvl[:, 0] + c * lvl[:, 1] + y, lvl[:, 2] + z + self.sensor_h], axis=1)
        keys = np.round(world * 4).astype(np.int64)
        for k, pt in zip(map(tuple, keys), world):
            if k not in self.seen:
                self.seen.add(k)
                self.pts.append(pt)
        if self.last_view is None or math.hypot(x - self.last_view[0], y - self.last_view[1]) > 0.2:
            self.views.append((x, y, z + 1.0))
            self.last_view = (x, y)

    def on_trigger(self, _req, resp):
        resp.message = self.run()
        resp.success = True
        return resp

    def run(self):
        if len(self.pts) < 500:
            return "not enough map points yet"
        pts, vps = np.array(self.pts), np.array(self.views)
        labels, nv = segment_points(pts, vps)
        out = dict(points=len(pts), voxels=nv, counts={c: int((labels == i).sum()) for i, c in enumerate(CLASSES)})
        if self.gt is not None:
            ev = evaluate_segmentation(labels, self.gt.classify(pts))
            out.update(accuracy=round(ev["accuracy"], 4), mean_iou=round(ev["mean_iou"], 4), iou={k: (None if v["iou"] is None else round(v["iou"], 3)) for k, v in ev["per_class"].items()})
        from std_msgs.msg import Header
        header = Header()
        header.stamp, header.frame_id = self.get_clock().now().to_msg(), "map"
        self.pub.publish(make_cloud(header, pts, COLORS[labels]))
        text = json.dumps(out)
        self.result_pub.publish(String(data=text))
        self.get_logger().info("segmentation: " + text)
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            with open(os.path.join(self.log_dir, "segmentation.json"), "w") as fh:
                fh.write(text)
        return text


def main():
    rclpy.init()
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
