"""Autonomous mapping + frontier exploration node (port of the web simulator's Mapper / Explorer / RobotController).

in : /lidar/points, /imu, pose source = /ground_truth/odom (default, mirrors the web 'shadow mode') or /slam/pose (pose_source:=slam)
out: /cmd_vel (Twist), /map (OccupancyGrid), /explore/path (Path), /explore/status (String, JSON)

Occupancy update: returns whose height above the local floor is in [obstacle_min, obstacle_max] mark obstacles; every ray carves free space
until it leaves that height band (so overhead pipes, ceilings and the floor never block the robot).
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

from .cloud_utils import level_points, pointcloud2_to_xyz
from .explorer_core import FREE, OCC, FrontierExplorer, OccupancyGrid as Grid, WaypointController, integrate_level_cloud
from .se2 import yaw_from_quat


class ExplorerNode(Node):
    def __init__(self):
        super().__init__("explorer")
        p = self.declare_parameter
        info = json.load(open(p("info_json", "").value))
        mask = json.load(open(p("truth_json", "").value))
        b = info["bounds"]
        self.grid = Grid(b["xMin"], b["xMax"], b["yMin"], b["yMax"], mask["res"])
        truth = np.fromfile(p("truth_bin", "").value, dtype=np.uint8).reshape(mask["H"], mask["W"]).astype(bool)
        self.home = (info["spawn"][0], info["spawn"][1])
        self.pose_source = p("pose_source", "ground_truth").value
        self.obs_min, self.obs_max = p("obstacle_min", 0.15).value, p("obstacle_max", 1.7).value
        self.sensor_h = p("sensor_height", 0.972).value
        self.ctl = WaypointController(v_max=p("v_max", 0.8).value, tol=p("waypoint_tolerance", 0.25).value)
        self.ex = FrontierExplorer(self.grid, self.ctl, self.home, truth_mask=truth, inflations=tuple(p("inflations", [0.5, 0.45, 0.4]).value),
                                   size_weight=p("size_weight", 0.12).value)
        self.pose = None            # (x, y, yaw) in the planning frame
        self.imu_q = None
        self.scans = 0
        self.last_t = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.path_pub = self.create_publisher(Path, "/explore/path", 1)
        self.status_pub = self.create_publisher(String, "/explore/status", 1)
        if self.pose_source == "slam":
            self.create_subscription(PoseStamped, "/slam/pose", lambda m: self._set(m.pose), 10)
        else:
            self.create_subscription(Odometry, "/ground_truth/odom", lambda m: self._set(m.pose.pose), 20)
        self.create_subscription(Imu, "/imu", lambda m: setattr(self, "imu_q", (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)), 20)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_slow)
        self.get_logger().info(f"explorer ready: pose source '{self.pose_source}', grid {self.grid.W}x{self.grid.H} @ {self.grid.res} m")

    def _set(self, pose):
        q = pose.orientation
        self.pose = (pose.position.x, pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_cloud(self, msg):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        lvl = level_points(pointcloud2_to_xyz(msg, max_range=25.0), self.imu_q)
        if len(lvl) == 0:
            return
        integrate_level_cloud(self.grid, x, y, yaw, lvl, self.sensor_h, self.obs_min, self.obs_max)
        self.scans += 1

    def tick(self):
        if self.pose is None or self.scans == 0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.2 if self.last_t is None else max(0.0, min(now - self.last_t, 1.0))
        self.last_t = now
        x, y, yaw = self.pose
        self.ex.update(dt, x, y)
        v, w = (0.0, 0.0) if self.ex.state == "complete" else self.ctl.step(x, y, yaw)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    def publish_slow(self):
        if self.scans == 0:
            return
        g, st = self.grid, self.grid.state()
        m = OccupancyGrid()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "map"
        m.info.resolution, m.info.width, m.info.height = float(g.res), int(g.W), int(g.H)
        m.info.origin.position.x, m.info.origin.position.y, m.info.origin.orientation.w = float(g.x_min), float(g.y_min), 1.0
        data = np.full(st.shape, -1, dtype=np.int8)
        data[st == FREE], data[st == OCC] = 0, 100
        m.data = data.ravel().tolist()
        self.map_pub.publish(m)
        path = Path()
        path.header = m.header
        for wx, wy in self.ex.path:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.orientation.w = float(wx), float(wy), 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=json.dumps(dict(state=self.ex.state, coverage=round(self.ex.coverage, 4), travelled=round(self.ex.travelled, 1),
                                                           frontiers=self.ex.frontiers, scans=self.scans))))
        self.get_logger().info(f"{self.ex.state}: coverage {self.ex.coverage * 100:.1f}% travelled {self.ex.travelled:.0f} m frontiers {self.ex.frontiers}",
                               throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()
