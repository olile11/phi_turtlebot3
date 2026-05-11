#! /usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped

ROBOT_FRAME = 'base_footprint'

class bot_control:
    def __init__(self, node) :
        self.node = node
        self.velocity_msg = TwistStamped()
        self.velocity_msg.header.frame_id = ROBOT_FRAME
        self.velocity_msg.twist.linear.y = 0.0
        self.velocity_msg.twist.linear.z = 0.0
        self.velocity_msg.twist.angular.x = 0.0
        self.velocity_msg.twist.angular.y = 0.0
        self.pub = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.P = 0.004

    #move function to move robot
    def move(self,linear,angular):
        self.velocity_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.velocity_msg.twist.linear.x = float(linear)
        self.velocity_msg.twist.angular.z = float(angular)
        self.pub.publish(self.velocity_msg)
        
    #fix error & bot position correction
    def fix_error(self, linear_error, orien_error):
        if orien_error < 0:           
            # fixing the yaw     
            self.move(0.2, self.P*orien_error + 0.6)
            self.node.get_logger().info("fixing yaw by turning left")

        elif orien_error > 0:           
            # fixing the yaw     
            self.move(0.2, self.P*orien_error)
            self.node.get_logger().info("fixing yaw by turning right")
                
        else:
            # moving in straight line
            self.move(0.4, 0.0)
            self.node.get_logger().info("moving straight")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.node.Node('bot_control_test_node')
    robot = bot_control(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
