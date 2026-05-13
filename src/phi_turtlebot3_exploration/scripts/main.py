#!/usr/bin/python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from scipy.ndimage import label


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return np.arctan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a: float) -> float:
    return np.arctan2(np.sin(a), np.cos(a))

MAX_VEL    = 0.1
MAX_OMEGA  = 1.5
CRUISE_VEL = 0.15

MAX_LIN_ACC = 0.4   # m/s²
MAX_ANG_ACC = 3.0   # rad/s²
CTRL_DT     = 0.05  # casado com o timer do _control_loop
THETA_DEADZONE = np.radians(2.0)

CMD_VEL_TOPIC = 'cmd_vel'
SCAN_TOPIC    = 'scan'
MAP_TOPIC     = '/map'

MAP_FRAME   = 'map'
ROBOT_FRAME = 'base_footprint'

GOAL_TOL          = 0.25
MIN_GOAL_DIST     = 1.0
MIN_FRONTIER_SIZE = 4
FREE_THRESHOLD    = 50
EXPLORE_SPIN      = 0.4

FORWARD_BIAS_MIN = 0.3

STUCK_WINDOW      = 5.0       # s
STUCK_DIST        = 0.05      # m

BLACKLIST_RADIUS  = 0.7       # m
BLACKLIST_TTL     = 30.0      # s

MAX_REPULSION     = 1.5


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

    def _cb_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid = np.isfinite(ranges)
        valid &= ranges > max(msg.range_min, 0.0)
        valid &= ranges < msg.range_max

        ranges = ranges[valid]
        angles = angles[valid]

        self.obstacle_pts = (ranges * np.cos(angles),
                             ranges * np.sin(angles))

    def _cb_map(self, msg: OccupancyGrid):
        H, W = msg.info.height, msg.info.width
        self.map_data       = np.array(msg.data, dtype=np.int8).reshape(H, W)
        self.map_resolution = msg.info.resolution
        self.map_origin_x   = msg.info.origin.position.x
        self.map_origin_y   = msg.info.origin.position.y

    def _update_pose(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, Time())
        except TransformException:
            return False
        self.x = tf.transform.translation.x
        self.y = tf.transform.translation.y
        q = tf.transform.rotation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.have_pose = True
        return True

    def find_frontier_goal(self):
        if self.map_data is None:
            return None

        grid    = self.map_data
        free    = (grid >= 0) & (grid < FREE_THRESHOLD)
        unknown = grid == -1

        adj_unknown = np.zeros_like(unknown, dtype=bool)
        adj_unknown[1:, :]  |= unknown[:-1, :]
        adj_unknown[:-1, :] |= unknown[1:, :]
        adj_unknown[:, 1:]  |= unknown[:, :-1]
        adj_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & adj_unknown
        if not np.any(frontier):
            return None

        labels, n = label(frontier, structure=np.ones((3, 3), dtype=int))
        if n == 0:
            return None

        sizes = np.bincount(labels.ravel())
        sizes[0] = 0

        rows, cols = np.where(frontier)
        wx = self.map_origin_x + (cols + 0.5) * self.map_resolution
        wy = self.map_origin_y + (rows + 0.5) * self.map_resolution
        dx = wx - self.x
        dy = wy - self.y
        d  = np.hypot(dx, dy)

        cluster_size = sizes[labels[rows, cols]].astype(float)

        now_ns = self.get_clock().now().nanoseconds
        self.blacklist = [b for b in self.blacklist if b[2] > now_ns]
        outside_blacklist = np.ones_like(d, dtype=bool)
        for bx, by, _ in self.blacklist:
            outside_blacklist &= np.hypot(wx - bx, wy - by) >= BLACKLIST_RADIUS

        far_enough = d >= MIN_GOAL_DIST
        big_enough = cluster_size >= MIN_FRONTIER_SIZE
        valid = big_enough & far_enough & outside_blacklist
        if not np.any(valid):
            valid = far_enough & outside_blacklist
        if not np.any(valid):
            valid = far_enough
        if not np.any(valid):
            return None

        ang_to_cell = np.arctan2(dy, dx)
        diff_ang = np.arctan2(np.sin(ang_to_cell - self.theta),
                              np.cos(ang_to_cell - self.theta))
        fwd = (np.cos(diff_ang) + 1.0) * 0.5
        bias = FORWARD_BIAS_MIN + (1.0 - FORWARD_BIAS_MIN) * fwd

        cost = np.where(
            valid,
            d / (np.log1p(cluster_size) * bias),
            np.inf,
        )
        i = int(np.argmin(cost))
        return float(wx[i]), float(wy[i])

    def _set_new_goal(self) -> bool:
        goal = self.find_frontier_goal()
        if goal is None:
            self.goal_x, self.goal_y = None, None
            self.get_logger().info(
                'Sem fronteiras visíveis — exploração concluída?',
                throttle_duration_sec=5.0)
            return False
        self.goal_x, self.goal_y = goal
        self.goal_count += 1

        self.stuck_ref_x    = self.x
        self.stuck_ref_y    = self.y
        self.stuck_ref_time = self.get_clock().now()

        self.get_logger().info(
            f'Goal #{self.goal_count} → ({self.goal_x:+.2f}, {self.goal_y:+.2f})')
        return True

    def _goal_watchdog(self):
        if self.map_data is None or not self.have_pose:
            return
        if self.goal_x is None:
            self._set_new_goal()
            return

        elapsed = (self.get_clock().now()
                   - self.stuck_ref_time).nanoseconds / 1e9
        if elapsed < STUCK_WINDOW:
            return

        moved = float(np.hypot(self.x - self.stuck_ref_x,
                               self.y - self.stuck_ref_y))
        if moved < STUCK_DIST:
            now_ns = self.get_clock().now().nanoseconds
            expiry = now_ns + int(BLACKLIST_TTL * 1e9)
            self.blacklist.append((self.goal_x, self.goal_y, expiry))
            self.get_logger().warn(
                f'Travado em ({self.goal_x:+.2f},{self.goal_y:+.2f}) — '
                f'andou {moved:.2f}m em {elapsed:.1f}s. Blacklist {BLACKLIST_TTL:.0f}s.')
            self._set_new_goal()
        else:
            self.stuck_ref_x    = self.x
            self.stuck_ref_y    = self.y
            self.stuck_ref_time = self.get_clock().now()

    def _control_loop(self):
        if not self._update_pose() or self.map_data is None:
            self._publish(0.0, 0.0)
            return

        if self.goal_x is None and not self._set_new_goal():
            self._publish(0.0, EXPLORE_SPIN)
            return

        d_goal = float(np.hypot(self.goal_x - self.x, self.goal_y - self.y))
        if d_goal < GOAL_TOL:
            self._set_new_goal()
            return

        f_res = self.get_attractive_force() + self.get_repulsive_force()
        if np.hypot(f_res[0], f_res[1]) < 1e-6:
            self._publish(0.0, 0.0)
            return

        theta_d   = np.arctan2(f_res[1], f_res[0])
        theta_err = wrap_angle(theta_d - self.theta)

        if abs(theta_err) < THETA_DEADZONE:
            theta_err = 0.0

        align = max(0.0, np.cos(theta_err)) ** 2
        slow  = np.tanh(d_goal / self.slow_radius)
        v_cmd = float(np.clip(CRUISE_VEL * align * slow, 0.0, MAX_VEL))
        w_cmd = float(np.clip(self.k_omega * theta_err, -MAX_OMEGA, MAX_OMEGA))
        self._publish(v_cmd, w_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            if node.cmd_vel_stamped:
                stop = TwistStamped()
                stop.header.stamp = node.get_clock().now().to_msg()
                stop.header.frame_id = ROBOT_FRAME
            else:
                stop = Twist()
            node.pub_cmd.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
