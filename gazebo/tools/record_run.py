"""Drive the robot with a fixed command pattern and record what the stack sees, for offline analysis (run INSIDE the container with ROS sourced,
while `ros2 launch mining_sim_gz sim.launch.py ... explore:=false` is running).

  python3 record_run.py <out.npz> <seconds> [forward_speed] [turn_rate]

Saves per LiDAR sweep: sensor-frame xyz, IMU quaternion, ground-truth pose (x, y, z, yaw), wheel-odometry pose, time.
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2

sys.path.insert(0, "/ws/src/mining_sim_gz")
from mining_sim_gz.cloud_utils import pointcloud2_to_xyz  # noqa: E402
from mining_sim_gz.se2 import yaw_from_quat  # noqa: E402


class Recorder(Node):
    def __init__(self, v, w):
        super().__init__("recorder")
        self.v, self.w = v, w
        self.gt = self.odom = self.imu = None
        self.rows = []
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: setattr(self, "gt", self._p(m, True)), 20)
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", self._p(m, False)), 20)
        self.create_subscription(Imu, "/imu", lambda m: setattr(self, "imu", (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)), 20)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.last_img_t = -1e9
        self.create_timer(0.1, lambda: self._drive())

    @staticmethod
    def _p(m, with_z):
        q = m.pose.pose.orientation
        return np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z, yaw_from_quat(q.x, q.y, q.z, q.w)])

    def _drive(self):
        if self.v != self.v:  # NaN speed = passive mode: only record, never publish commands
            return
        t = Twist()
        t.linear.x, t.angular.z = float(self.v), float(self.w)
        self.cmd.publish(t)

    def on_image(self, msg):
        # save a camera frame every 4 s as a binary PPM (converted to PNG on the host)
        now = time.time()
        if now - self.last_img_t < 10.0 or msg.encoding != "rgb8":
            return
        self.last_img_t = now
        import os
        os.makedirs("/out/frames", exist_ok=True)
        with open(f"/out/frames/frame_{int(now)}.ppm", "wb") as fh:
            fh.write(b"P6 %d %d 255\n" % (msg.width, msg.height) + bytes(msg.data))

    def on_cloud(self, msg):
        if self.gt is None or self.odom is None or self.imu is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = pointcloud2_to_xyz(msg)
        self.rows.append((t, raw.astype(np.float32), np.array(self.imu), self.gt.copy(), self.odom.copy(), msg.width * msg.height))


def main():
    out, seconds = sys.argv[1], float(sys.argv[2])
    v = float("nan") if len(sys.argv) > 3 and sys.argv[3] == "passive" else (float(sys.argv[3]) if len(sys.argv) > 3 else 0.5)
    w = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    rclpy.init()
    node = Recorder(v, w)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.v == node.v:
        node.v = node.w = 0.0
        for _ in range(10):
            node._drive()
            time.sleep(0.05)
    rows = node.rows
    np.savez_compressed(out, t=np.array([r[0] for r in rows]), imu=np.array([r[2] for r in rows]), gt=np.array([r[3] for r in rows]),
                        odom=np.array([r[4] for r in rows]), n_raw=np.array([r[5] for r in rows]),
                        xyz=np.array([r[1] for r in rows], dtype=object), allow_pickle=True)
    print(f"recorded {len(rows)} sweeps -> {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
