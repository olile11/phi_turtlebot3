#! /usr/bin/env python3
import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from controller import ModelControl


CAMERA_TOPIC = '/camera/image_raw'
DEBUG_TOPIC = '/line_follower/debug_image'

LINEAR_STRAIGHT  = 0.20  
LINEAR_CURVE = 0.07   
ERROR_FULL_CURVE = 50     
K_ANGULAR = 0.01
DEADBAND_PX = 3


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = cv_bridge.CvBridge()

        self.sub = self.create_subscription(Image, CAMERA_TOPIC, self.callback, 10)
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 10)
        self.controller = ModelControl(self)

    def _crop_bottom(self, frame):
        h = frame.shape[0]
        return frame[int(h * 0.7):h, :]

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        roi = self._crop_bottom(frame)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv, 9)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 100]))

        h_roi, w_roi = mask.shape
        goal_x = w_roi // 2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if not contours:
            self.controller.move(0.0, 0.0)
            self._publish_debug(roi, mask, None, goal_x)
            return

        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] == 0:
            self.controller.move(0.0, 0.0)
            self._publish_debug(roi, mask, None, goal_x)
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        angular_eps = cx - goal_x
        if abs(angular_eps) < DEADBAND_PX:
            angular = 0.0
        else:
            angular = -K_ANGULAR * angular_eps

        t = min(abs(angular_eps) / ERROR_FULL_CURVE, 1.0)
        linear = LINEAR_STRAIGHT * (1.0 - t) + LINEAR_CURVE * t

        self.controller.move(linear, angular)
        self.get_logger().info(
            f"err ang={angular_eps:+d}px | lin={linear:.2f}  ang={angular:+.3f}", throttle_duration_sec=0.5)

        self._publish_debug(roi, mask, (cx, cy, c), goal_x)

    def _publish_debug(self, roi, mask, centroid, goal_x):
        if self.debug_pub.get_subscription_count() == 0:
            return
        debug = roi.copy()
        debug[mask < 100] = (0, 255, 0)
        cv2.line(debug, (goal_x, 0), (goal_x, debug.shape[0]), (0, 0, 255), 2)
        if centroid is not None:
            cx, cy, contour = centroid
            cv2.drawContours(debug, [contour], -1, (0, 255, 255), 1)
            cv2.putText(debug, 'X', (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
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
