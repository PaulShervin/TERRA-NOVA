"""Wheel + gyro odometry for the SLAM stack (/odom_noisy and TF odom->base_link).

Distance comes from Gazebo's differential-drive wheel odometry (/odom) with the simulated wheel errors of the web simulator (scale error,
random walk). HEADING comes from integrating the IMU gyro (/imu angular_velocity.z): on the rough cave floor the wheels slip during turns, so
wheel-derived heading is badly wrong (measured: 446 deg reported vs 332 deg real), which is why real robots use a gyro. The gyro carries its own
noise and bias (set in the SDF), so heading still drifts slowly. Ground truth is never used here.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from .se2 import between, compose, quat_from_yaw, yaw_from_quat
from .slam_core import odometry_noise_model


class OdomNoiseNode(Node):
    def __init__(self):
        super().__init__("odom_noise")
        p = self.declare_parameter
        self.rng = np.random.default_rng(p("seed", 1).value)
        self.params = dict(scale_error=p("scale_error", 0.02).value, trans_sigma=p("trans_sigma", 0.015).value,
                           yaw_scale=0.0, yaw_bias_per_m=0.0, yaw_sigma=0.0)     # heading error comes from the gyro, not from a model
        self.use_gyro = p("use_gyro", True).value
        self.prev = None
        self.noisy = np.zeros(3)
        self.gyro_yaw, self.gyro_t = 0.0, None
        self.gyro_yaw_at_prev = 0.0
        self.pub = self.create_publisher(Odometry, "/odom_noisy", 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Imu, "/imu", self.on_imu, 100)

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.gyro_t is not None and 0 < t - self.gyro_t < 0.5:
            self.gyro_yaw += msg.angular_velocity.z * (t - self.gyro_t)
        self.gyro_t = t

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        cur = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w)])
        if self.prev is not None:
            delta = odometry_noise_model(between(self.prev, cur), self.rng, **self.params)
            if self.use_gyro:
                delta[2] = self.gyro_yaw - self.gyro_yaw_at_prev
            self.noisy = compose(self.noisy, delta)
        self.prev, self.gyro_yaw_at_prev = cur, self.gyro_yaw
        out = Odometry()
        out.header.stamp, out.header.frame_id, out.child_frame_id = msg.header.stamp, "odom", "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y = float(self.noisy[0]), float(self.noisy[1])
        qx, qy, qz, qw = quat_from_yaw(self.noisy[2])
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = qx, qy, qz, qw
        out.twist = msg.twist
        self.pub.publish(out)
        t = TransformStamped()
        t.header = out.header
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y = float(self.noisy[0]), float(self.noisy[1])
        t.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
