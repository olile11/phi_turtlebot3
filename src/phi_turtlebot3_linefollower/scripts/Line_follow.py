#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class SpeedController(Node):
    def __init__(self):
        super().__init__("speed_controller")
        
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0

        self.sub = self.create_subscription(Odometry, "/odometry/filtered", self.newOdom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.timer = self.create_timer(0.25, self.timer_callback) # 4 Hz = 0.25 seconds

    def newOdom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

    def timer_callback(self):
        vel = Twist()
        vel.linear.x = 0.5
        vel.angular.z = 0.0
        self.pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
