#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotDriver(Node):
  def __init__(self):
    super().__init__('robot_driver')
    self.publisher_ = self.create_publisher(Twist, 'tri_cont/cmd_vel', 10)
    self.time_period = 0.1
    self.timer = self.create_timer(self.time_period, self.timer_callback)

  def timer_callback(self):
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.0
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)

def main(args = None):
  rclpy.init(args = args)
  robot_driver = RobotDriver()
  rclpy.spin(robot_driver)
  robot_driver.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()