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

GOAL_TOL          = 0.25      # raio de chegada (m)
MIN_GOAL_DIST     = 1.0       # distância mínima do goal — força commit longo
MIN_FRONTIER_SIZE = 4         # nº mínimo de células num cluster
FREE_THRESHOLD    = 50        # célula livre se valor < FREE_THRESHOLD
EXPLORE_SPIN      = 0.4       # ω quando não há fronteira

# Bias frontal: peso ao escolher célula no setor à frente do robô.
# Multiplicador do cost: 1.0 à frente, FORWARD_BIAS_MIN atrás.
# Quanto menor FORWARD_BIAS_MIN, mais o robô evita meia-volta.
FORWARD_BIAS_MIN = 0.3

# Detector de travado: se em STUCK_WINDOW segundos o robô andou menos
# que STUCK_DIST metros e ainda há goal pendente, assume bloqueio por
# obstáculo e reescolhe a fronteira. Em fluxo livre o goal não muda.
STUCK_WINDOW      = 5.0       # s
STUCK_DIST        = 0.05      # m

# Quando travamos num goal, esse ponto entra numa blacklist por
# BLACKLIST_TTL segundos: nenhum goal novo pode cair a menos de
# BLACKLIST_RADIUS dele. Força o explorador a tentar outra direção.
BLACKLIST_RADIUS  = 0.7       # m
BLACKLIST_TTL     = 30.0      # s

# Limite na magnitude da repulsão. Sem isso, paredes a < 0.3 m criam
# forças >> atração e o robô fica num poço de potencial.
MAX_REPULSION     = 1.5


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # Campo potencial.
        self.ka      = 0.7
        self.kr      = 0.05
        self.d_star  = 0.5

        # Controlador.
        self.k_omega     = 1.0
        self.slow_radius = 0.5

        # Estado para slew-rate dos comandos.
        self.v_prev = 0.0
        self.w_prev = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_pose = False

        self.obstacle_pts = (np.array([]), np.array([]))

        self.map_data       = None
        self.map_origin_x   = 0.0
        self.map_origin_y   = 0.0
        self.map_resolution = 0.05

        self.goal_x = None
        self.goal_y = None
        self.goal_count = 0

        # Amostra para o detector de "travado".
        self.stuck_ref_x    = 0.0
        self.stuck_ref_y    = 0.0
        self.stuck_ref_time = self.get_clock().now()

        # [(x, y, expiry_ns), ...] — goals onde o robô travou.
        self.blacklist = []

        # Sim (gz_bridge) usa TwistStamped; TB3 real (turtlebot3_node) usa Twist.
        self.cmd_vel_stamped = bool(
            self.declare_parameter('cmd_vel_stamped', True).value)
        if self.cmd_vel_stamped:
            self.twist = TwistStamped()
            self.twist.header.frame_id = ROBOT_FRAME
            cmd_vel_type = TwistStamped
        else:
            self.twist = Twist()
            cmd_vel_type = Twist

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # /map em nav2/slam_toolbox é TRANSIENT_LOCAL (latched).
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_cmd = self.create_publisher(cmd_vel_type, CMD_VEL_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._cb_scan, sensor_qos)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._cb_map, map_qos)

        self.create_timer(0.05, self._control_loop)
        self.create_timer(2.0,  self._goal_watchdog)

        self.get_logger().info(
            'FrontierExplorer iniciado\n'
            f'  cmd_vel={CMD_VEL_TOPIC} scan={SCAN_TOPIC} map={MAP_TOPIC}\n'
            f'  TF: {MAP_FRAME} → {ROBOT_FRAME}'
        )

    # ── Callbacks ────────────────────────────────────────────────

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

    def get_attractive_force(self) -> np.ndarray:
        return self.ka * np.array([self.goal_x - self.x,
                                   self.goal_y - self.y])

    def get_repulsive_force(self) -> np.ndarray:
        rep = np.zeros(2)
        xs_l, ys_l = self.obstacle_pts
        if xs_l.size == 0:
            return rep

        d_local = np.hypot(xs_l, ys_l)
        i = int(np.argmin(d_local))
        d_min = float(d_local[i])
        if d_min < 1e-3 or d_min >= self.d_star:
            return rep

        cos_t, sin_t = np.cos(self.theta), np.sin(self.theta)
        ox = self.x + xs_l[i] * cos_t - ys_l[i] * sin_t
        oy = self.y + xs_l[i] * sin_t + ys_l[i] * cos_t

        dx, dy = self.x - ox, self.y - oy
        scalar = self.kr * (1.0 / d_min - 1.0 / self.d_star) / (d_min ** 3)
        rep = scalar * np.array([dx, dy])

        # Cap: paredes a < 0.3 m geram forças absurdas que dominam a
        # atração e prendem o robô. Limita pela magnitude.
        mag = float(np.hypot(rep[0], rep[1]))
        if mag > MAX_REPULSION:
            rep *= MAX_REPULSION / mag
        return rep

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
        sizes[0] = 0   # background

        rows, cols = np.where(frontier)
        wx = self.map_origin_x + (cols + 0.5) * self.map_resolution
        wy = self.map_origin_y + (rows + 0.5) * self.map_resolution
        dx = wx - self.x
        dy = wy - self.y
        d  = np.hypot(dx, dy)

        cluster_size = sizes[labels[rows, cols]].astype(float)

        # Blacklist: descarta entradas expiradas e exclui células
        # próximas dos goals travados.
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

        # Bias frontal: ângulo da célula relativo à heading do robô.
        # cos(diff) = +1 à frente, -1 atrás. Mapeia para [FORWARD_BIAS_MIN, 1].
        ang_to_cell = np.arctan2(dy, dx)
        diff_ang = np.arctan2(np.sin(ang_to_cell - self.theta),
                              np.cos(ang_to_cell - self.theta))
        fwd = (np.cos(diff_ang) + 1.0) * 0.5         # 0..1
        bias = FORWARD_BIAS_MIN + (1.0 - FORWARD_BIAS_MIN) * fwd

        # Custo: distância / (info_gain × bias). Quanto menor, melhor.
        # Resultado: prefere clusters grandes, à frente, em distância média.
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
        # Reseta a janela de "travado" — começa a medir progresso a partir
        # de agora, com a posição atual como referência.
        self.stuck_ref_x    = self.x
        self.stuck_ref_y    = self.y
        self.stuck_ref_time = self.get_clock().now()
        self.get_logger().info(
            f'Goal #{self.goal_count} → ({self.goal_x:+.2f}, {self.goal_y:+.2f})')
        return True

    def _goal_watchdog(self):
        """Reescolhe goal só quando o robô estiver de fato bloqueado."""
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
            # Bloqueia esta célula por BLACKLIST_TTL: novos goals não
            # podem cair perto dela. Quebra o ciclo de re-escolher
            # exatamente o mesmo ponto inalcançável.
            now_ns = self.get_clock().now().nanoseconds
            expiry = now_ns + int(BLACKLIST_TTL * 1e9)
            self.blacklist.append((self.goal_x, self.goal_y, expiry))
            self.get_logger().warn(
                f'Travado em ({self.goal_x:+.2f},{self.goal_y:+.2f}) — '
                f'andou {moved:.2f}m em {elapsed:.1f}s. Blacklist {BLACKLIST_TTL:.0f}s.')
            self._set_new_goal()
        else:
            # Houve progresso; rearma a janela.
            self.stuck_ref_x    = self.x
            self.stuck_ref_y    = self.y
            self.stuck_ref_time = self.get_clock().now()

    @staticmethod
    def _slew(target: float, prev: float, max_step: float) -> float:
        return prev + float(np.clip(target - prev, -max_step, max_step))

    def _publish(self, v_cmd: float, w_cmd: float):
        # Slew-rate: limita variação de comando entre ticks → movimento suave.
        v = self._slew(v_cmd, self.v_prev, MAX_LIN_ACC * CTRL_DT)
        w = self._slew(w_cmd, self.w_prev, MAX_ANG_ACC * CTRL_DT)
        self.v_prev, self.w_prev = v, w

        if self.cmd_vel_stamped:
            self.twist.header.stamp    = self.get_clock().now().to_msg()
            self.twist.twist.linear.x  = v
            self.twist.twist.angular.z = w
        else:
            self.twist.linear.x  = v
            self.twist.angular.z = w
        self.pub_cmd.publish(self.twist)

    def _control_loop(self):
        if not self._update_pose() or self.map_data is None:
            self._publish(0.0, 0.0)
            return

        if self.goal_x is None and not self._set_new_goal():
            # Sem fronteira válida — gira no lugar para o SLAM expandir
            # o mapa (slam_toolbox processa nova scan a cada ~5.7°).
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
