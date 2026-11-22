"""LiDAR odometry + pose-graph SLAM node (shadow mode: scored against Gazebo's ground truth, it does not drive the robot unless
the explorer is started with pose_source:=slam).

in : /lidar/points (PointCloud2), /odom_noisy (nav_msgs/Odometry), /imu (sensor_msgs/Imu, level correction),
     /ground_truth/odom (only for the error metrics)
out: /slam/pose (PoseStamped, map frame), TF map->odom, /slam/path /slam/odom_path /slam/gt_path (nav_msgs/Path),
     /slam/loop_closures (MarkerArray), /slam/metrics (std_msgs/String, JSON); metrics.json is written to `log_dir` on shutdown.
"""
import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .cloud_utils import level_points, pointcloud2_to_xyz, scan_points_2d_vertical
from .se2 import between, compose, quat_from_yaw, yaw_from_quat
from .slam_core import SlamEstimator


class ScanSlamNode(Node):
    def __init__(self):
        super().__init__("scan_slam")
        p = self.declare_parameter
        info = json.load(open(p("info_json", "").value))
        self.spawn = np.array([info["spawn"][0], info["spawn"][1], info["yaw"]], dtype=float)
        self.min_rel, self.max_rel = p("band_min", 0.25).value, p("band_max", 2.0).value
        self.log_dir = p("log_dir", "").value
        self.slam = SlamEstimator(scan_matching=p("scan_matching", False).value)
        self.slam.init(self.spawn)
        self.odom_latest = self.odom_prev = None
        self.imu_q = None
        self.gt = None
        self.t0 = None
        self.pose_pub = self.create_publisher(PoseStamped, "/slam/pose", 10)
        self.path_pubs = {k: self.create_publisher(Path, f"/slam/{k}", 1) for k in ("path", "odom_path", "gt_path")}
        self.marker_pub = self.create_publisher(MarkerArray, "/slam/loop_closures", 1)
        self.metrics_pub = self.create_publisher(String, "/slam/metrics", 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom_noisy", lambda m: setattr(self, "odom_latest", self._pose(m)), 20)
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: setattr(self, "gt", self._pose(m)), 20)
        self.create_subscription(Imu, "/imu", lambda m: setattr(self, "imu_q", (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)), 20)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_cloud, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_slow)
        self.get_logger().info(f"scan_slam ready (start pose {self.spawn.round(2).tolist()})")

    @staticmethod
    def _pose(m):
        q = m.pose.pose.orientation
        return np.array([m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w)])

    def on_cloud(self, msg):
        if self.odom_latest is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.t0 = t if self.t0 is None else self.t0
        delta = np.zeros(3) if self.odom_prev is None else between(self.odom_prev, self.odom_latest)
        self.odom_prev = self.odom_latest.copy()
        lvl = level_points(pointcloud2_to_xyz(msg, max_range=25.0), self.imu_q)
        pts = scan_points_2d_vertical(lvl)
        self.slam.update(delta, pts, t - self.t0, truth=self.gt)
        est = self.slam.pose
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = msg.header.stamp, "map"
        ps.pose.position.x, ps.pose.position.y = float(est[0]), float(est[1])
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat_from_yaw(est[2])
        self.pose_pub.publish(ps)
        # map -> odom = est (+) inverse(odom_latest)
        mo = compose(est, between(self.odom_latest, np.zeros(3)))
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = msg.header.stamp, "map", "odom"
        tf.transform.translation.x, tf.transform.translation.y = float(mo[0]), float(mo[1])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quat_from_yaw(mo[2])
        self.tf.sendTransform(tf)

    def _path(self, xy, stamp):
        path = Path()
        path.header.stamp, path.header.frame_id = stamp, "map"
        for x, y in xy:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def publish_slow(self):
        n = len(self.slam.samples)
        if n < 2:
            return
        stamp = self.get_clock().now().to_msg()
        stride = max(1, n // 1500)
        self.path_pubs["path"].publish(self._path(self.slam.trajectory(stride), stamp))
        self.path_pubs["odom_path"].publish(self._path(self.slam.odometry_trajectory(stride), stamp))
        self.path_pubs["gt_path"].publish(self._path(self.slam.truth_trajectory(stride), stamp))
        m = Marker()
        m.header.stamp, m.header.frame_id, m.ns, m.id, m.type, m.action = stamp, "map", "loop_closures", 0, Marker.LINE_LIST, Marker.ADD
        m.scale.x, m.color.r, m.color.g, m.color.b, m.color.a = 0.08, 1.0, 0.82, 0.23, 1.0
        from geometry_msgs.msg import Point
        for c in self.slam.loop_closures:
            for k in (c["frm"], c["to"]):
                a = self.slam.kfs[k].pose
                m.points.append(Point(x=float(a[0]), y=float(a[1]), z=0.5))
        self.marker_pub.publish(MarkerArray(markers=[m]))
        metrics = self.slam.metrics()
        self.metrics_pub.publish(String(data=json.dumps(metrics)))
        self.write_report(metrics)   # written every second: SIGINT from ros2 launch does not reliably reach the shutdown path
        self.get_logger().info("ATE slam %.2f m / odom-only %.2f m | loops %d (rejected %d) | keyframes %d" % (
            metrics.get("ate", float("nan")), metrics.get("ate_odom", float("nan")), metrics["loop_closures"], metrics["rejected_loops"], metrics["keyframes"]),
            throttle_duration_sec=10.0)

    def write_report(self, metrics=None):
        if not self.log_dir:
            return
        os.makedirs(self.log_dir, exist_ok=True)
        with open(os.path.join(self.log_dir, "slam_metrics.json"), "w") as fh:
            json.dump(metrics if metrics is not None else self.slam.metrics(), fh, indent=2)


def main():
    rclpy.init()
    node = ScanSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.write_report()
    node.destroy_node()
    rclpy.try_shutdown()
