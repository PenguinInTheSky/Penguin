#!/usr/bin/env python3

import yaml
from PIL import Image
import numpy
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from scipy.spatial.transform import Rotation
import math

BLOCKED = 0
EMPTY = 254

ROBOT_COMFORT_RADIUS = 0.8

# get map
pkg_path = os.path.join(get_package_share_directory('Penguin'))
pgm_file = os.path.join(pkg_path, 'maps', 'small_room', 'small_room_saved.pgm')

image = Image.open(pgm_file)
image_array = numpy.array(image)

yaml_file = os.path.join(pkg_path, 'maps', 'small_room', 'small_room_saved.yaml')
with open(yaml_file, 'r') as file:
  map_info = yaml.safe_load(file)

map_width, map_height = image.size

map_origin = map_info['origin']
map_resolution = map_info['resolution']

visited = numpy.zeros([map_width, map_height], dtype = bool)


class InitialPosePublisher(Node):
  def __init__(self):
    super().__init__('initial_pose_publisher')
    self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
    
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = self.get_clock().now().to_msg()
    initial_pose.header.frame_id = 'map'

    initial_pose.pose.pose.position.x = 0.0
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.position.z = 0.0

    # Set the initial orientation (as a quaternion)
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0

    # Set the covariance (this example assumes some default values)
    initial_pose.pose.covariance = [0.0] * 36  # Modify covariance as needed

    # Publish the initial pose
    self.publisher_.publish(initial_pose)
    self.get_logger().info('Initial pose published.')
    
class RobotDriver(Node):
  def __init__(self):
    super().__init__('robot_driver')

    self.subscription = self.create_subscription(
      PoseWithCovarianceStamped,
      '/amcl_pose',
      self.pose_callback,
      10
    )

    self.publisher_ = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
    self.moving_rate = 1.0
    self.timer = self.create_timer(self.moving_rate, self.move)

    self.current_pose = Pose()

    self.get_logger().info("Created robot driver")

  def pose_callback(self, msg):
    self.current_pose = msg.pose.pose
    # self.mark_visited()
    # self.print_visited_map()
    self.real_to_map_position(self.current_position())
    self.get_logger().info('Current pose is: "%s"' % self.current_pose)

  def get_message(self, x, y, z, roll, pitch, yaw):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.x = roll
    msg.angular.y = pitch
    msg.angular.z = yaw
    return msg

  def move_ahead(self):
    msg = self.get_message(1/self.moving_rate, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)
    self.get_logger().info('Position: "%s"' % self.current_position())

  def move_backward(self):
    msg = self.get_message(-1/self.moving_rate, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)
  
  def mark_visited(self):
    MAP_COMFORT_RADIUS = int(ROBOT_COMFORT_RADIUS / map_resolution)
    robot_x, robot_y = self.real_to_map_position(self.current_position())
    # self.get_logger().info('Robot map x: "%.2f"' % robot_x)
    # self.get_logger().info('Robot map y: "%.2f"' % robot_y)
    for x in range (robot_x - MAP_COMFORT_RADIUS, robot_x + MAP_COMFORT_RADIUS):
      for y in range (robot_y - MAP_COMFORT_RADIUS, robot_y + MAP_COMFORT_RADIUS):
        visited[x, y] = True
  
  # DEBUG
  def print_visited_map(self):
    for x in range(0, map_width):
      for y in range(0, map_height):
        if(visited[x, y]):
          print(1, end = "")
        else:
          print(0, end="")
      print("\n")

  # # is left side of the robot on the map, not relatively to the robot, unvisited
  def is_left_side_unvisited(self):
    left_pose = (self.current_position()[0], self.current_position()[1] + ROBOT_COMFORT_RADIUS)
    left_map = self.real_to_map_position(left_pose)
    return visited[left_map[0], left_map[1]]

  # def is_facing_left(self):
  #   return None

  # # is right side of the robot on the map, not relatively to the robot, unvisited
  # def is_right_side_unvisited(self):
  #   return False
  
  # def is_facing_right(self):
  #   return None
  

  # def face_left(self):
  #   return None
  
  # def face_right(self):
  #   return None
  
  # def face_ahead(self):
  #   return None
    
  # def move_left_unvisited(self):
  #   if not self.is_facing_left():
  #     self.face_left()
  #   if not self.blocked_ahead():
  #     self.move_ahead()
  #   else:
  #     self.face_ahead()

  # def change_line(self):
  #   self.face_right()
  #   self.move_ahead()
  #   self.face_right()


    
  # def face_right(self):
  #   return None

  # def blocked_ahead(distance):
  #   return None

  # def move_forward():
  #   return None

  # def next_line():
  #   return None

  # def go_to_starting_position():
  #   return None

  # def finish():
  #   return None

  def current_orientation(self):
    return Rotation.from_quat(self.current_pose.orientation).as_euler('xyz', degrees = False)

  def current_position(self):
    return self.current_pose.position
  
  def real_to_map_position(self, pos):
    self.get_logger().info('Robot x: "%.2f"' % pos.x)
    self.get_logger().info('Robot y: "%.2f"' % pos.y)
    x = int((pos.x - map_origin[0]) / map_resolution)
    y = map_height - int((pos.y - map_origin[1]) / map_resolution)
    self.get_logger().info('Robot map x: "%.2f"' % x)
    self.get_logger().info('Robot map y: "%.2f"' % y)
    return (x, y)
  
  def move(self):
    return None
    self.move_ahead()
    # if self.is_left_side_unvisited():
    #   self.move_left_unvisited()
    # elif not self.blocked_ahead():
    #   self.move_ahead()
    # elif self.is_right_side_unvisited():
    #   self.change_line()
    # else:
    #   self.move_backward()
  
def main(args=None):
    rclpy.init(args=args)

    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin_once(initial_pose_publisher, timeout_sec=2.0)

    driver = RobotDriver()
    rclpy.spin(driver)
    
    initial_pose_publisher.destroy_node()
    driver.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

# for x in range(0, 156):
#   for y in range(0, 102):
#     pixel = image.getpixel((x, y))
#     to_print = "E"
#     if pixel == EMPTY:
#       to_print = "o"
#     elif pixel != BLOCKED :
#       to_print = "?"
#     print(to_print, end = "")
#   print("\n")

# print(f"Image size: {image.size}")
# print(f"Image mode: {image.mode}")
# print("Pixel data:")
# print(image_array)

# get current robot's position

# robot goes to initial pose (corner of the room)

# robot goes back and forth in a line



# robot moves to next line