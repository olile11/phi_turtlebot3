#! /usr/bin/env python3
"""Janela única OpenCV com a câmera raw (esq) + segmentação debug (dir).

Roda como nó separado: cv2.imshow + waitKey ficam na main thread (não no
callback). É por isso que o line_follower não pode chamar cv2.imshow lá
— travava o executor e o ros2 launch escalava p/ SIGKILL no shutdown.
"""
import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


RAW_TOPIC = '/camera/image_raw'
DEBUG_TOPIC = '/line_follower/debug_image'
WINDOW = 'line_follower view'


class Viewer(Node):
    def __init__(self):
        super().__init__('line_follower_viewer')
        self.bridge = cv_bridge.CvBridge()
        self.raw = None
        self.debug = None
        self.create_subscription(Image, RAW_TOPIC, self._on_raw, 10)
        self.create_subscription(Image, DEBUG_TOPIC, self._on_debug, 10)

    def _on_raw(self, msg):
        self.raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_debug(self, msg):
        self.debug = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def render(self):
        if self.raw is None and self.debug is None:
            return None
        h, w = (self.raw.shape[:2] if self.raw is not None
                else (240, 320))
        left = (self.raw if self.raw is not None
                else np.zeros((h, w, 3), dtype=np.uint8))
        right = np.zeros((h, w, 3), dtype=np.uint8)
        if self.debug is not None:
            # debug = ROI inferior (40% de baixo); coloca no fundo do canvas
            # p/ manter o alinhamento vertical com a raw.
            dh, dw = self.debug.shape[:2]
            dh = min(dh, h)
            dw = min(dw, w)
            right[h - dh:h, :dw] = self.debug[:dh, :dw]
        return np.hstack([left, right])


def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            frame = node.render()
            if frame is not None:
                cv2.imshow(WINDOW, frame)
            # waitKey na main thread — única chamada cv2 fora dos callbacks.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
