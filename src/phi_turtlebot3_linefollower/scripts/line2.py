#! /usr/bin/env python3
import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_controller import bot_control


CAMERA_TOPIC = '/camera/image_raw'
DEBUG_TOPIC = '/line_follower/debug_image'

# Controle proporcional. Angular = -K * erro_pixel (sinal oposto: erro>0
# (linha à direita) → angular<0 → giro horário). Linear interpola entre
# LINEAR_STRAIGHT (reta, erro≈0) e LINEAR_CURVE (curva forte, |erro|≥ERROR_FULL_CURVE).
LINEAR_STRAIGHT  = 0.14   # m/s — velocidade na reta
LINEAR_CURVE     = 0.05   # m/s — velocidade dentro da curva
ERROR_FULL_CURVE = 60     # px — erro a partir do qual usa LINEAR_CURVE
K_ANGULAR        = 0.006  # rad/s por pixel de erro
DEADBAND_PX      = 5


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = cv_bridge.CvBridge()

        self.sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.callback, 10)
        # Frame anotado (linha + centróide + alvo) p/ rqt_image_view.
        # Sem cv2.imshow no callback — bloqueia o executor e gera SIGKILL
        # no shutdown do ros2 launch.
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 10)

        self.bc = bot_control(self)

    def _crop_bottom(self, frame):
        # Faixa inferior da imagem (mais próxima do robô). Câmera do
        # burger_cam é 320x240; corta os 40% de baixo.
        h = frame.shape[0]
        return frame[int(h * 0.6):h, :]

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        roi = self._crop_bottom(frame)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv, 9)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 65]))

        h_roi, w_roi = mask.shape
        target_x = w_roi // 2

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if not contours:
            # Sem linha: para. (fix_error(0,0) caía em move(0.4, 0.0) e
            # disparava o robô pra fora da pista.)
            self.bc.move(0.0, 0.0)
            self._publish_debug(roi, mask, None, target_x)
            return

        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] == 0:
            self.bc.move(0.0, 0.0)
            self._publish_debug(roi, mask, None, target_x)
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Erro em pixels: negativo = linha à esquerda, positivo = à direita.
        error_x = cx - target_x
        if abs(error_x) < DEADBAND_PX:
            angular = 0.0
        else:
            angular = -K_ANGULAR * error_x  # sinal oposto = vira em direção à linha

        # Linear: rápido na reta, desacelera proporcional à magnitude do erro.
        t = min(abs(error_x) / ERROR_FULL_CURVE, 1.0)
        linear = LINEAR_STRAIGHT * (1.0 - t) + LINEAR_CURVE * t

        self.bc.move(linear, angular)
        self.get_logger().info(
            f"err={error_x:+d}px  lin={linear:.2f}  ang={angular:+.3f}",
            throttle_duration_sec=0.5)

        self._publish_debug(roi, mask, (cx, cy, c), target_x)

    def _publish_debug(self, roi, mask, centroid, target_x):
        # Compõe ROI BGR + máscara em verde + contorno + centróide + alvo.
        # Subscribers (rqt_image_view) não custam nada se ninguém escuta.
        if self.debug_pub.get_subscription_count() == 0:
            return
        debug = roi.copy()
        debug[mask > 0] = (0, 255, 0)
        cv2.line(debug, (target_x, 0), (target_x, debug.shape[0]),
                 (255, 0, 0), 1)
        if centroid is not None:
            cx, cy, contour = centroid
            cv2.drawContours(debug, [contour], -1, (0, 255, 255), 1)
            cv2.circle(debug, (cx, cy), 4, (0, 0, 255), -1)
        msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
