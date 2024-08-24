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

ROBOT_COMFORT_RADIUS = 0.7
ROBOT_COVER_RADIUS = 0.5

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

for x in range(0, map_width):
  for y in range(0, map_height):
    pixel = image.getpixel((x, y))
    if pixel != EMPTY:
      visited[x, y] = True


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
    self.publish_rate = 1
    self.timer = self.create_timer(self.publish_rate, self.move)
    self.current_pose = Pose()
    self.frozen = False
    self.angular_velocity = math.pi/6
    self.angular_precision = 0.03
    self.linear_velocity = 0.7

    self.get_logger().info("Created robot driver")

  def pose_callback(self, msg):
    self.current_pose = msg.pose.pose
    self.mark_visited()
    self.print_visited_map()
    self.real_to_map_position(self.pos_to_tuple(self.current_position()))
    # self.get_logger().info('Left_side_unvisited is "%s' % self.is_left_side_unvisited())
    # self.get_logger().info('Current posing is: "%s"' % self.current_position())
    # self.get_logger().info('Current orientation is: "%s"' % self.current_orientation())

  def get_message(self, x, y, z, roll, pitch, yaw):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.x = roll
    msg.angular.y = pitch
    msg.angular.z = yaw
    return msg

  def is_out_of_bound(self, pos):
    return pos[0] < 0 or pos[1] < 0 or pos[0] > map_width or pos[1] > map_height
  
  # return true if 2 floats are equal to a certain precision
  def equal_floats(self, float_main, float_other, precision):
    if (float_main == 0):
      return float_main == float_other
    return abs((float_other - float_main) / float_main) <= precision
  
  # cmp_code: 0 = equal, 1 = main >= other, 2 = main > other, -1 = main <= other, -2 = main < other
  def compare_floats(self, float_main, float_other, precision, cmp_code):
    equal = self.equal_floats(float_main, float_other, precision)
    if equal:
      if cmp_code == 0 or cmp_code == 1 or cmp_code == -1:
        return True
      else:
        return False
    else:
      if float_main > float_other:
        return cmp_code == 2 or cmp_code == 1
      else:
        return cmp_code == -2 or cmp_code == -1

  def freeze(self):
    msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.frozen = True
    self.get_logger().info('Freeze robot')

  def move_ahead(self):
    msg = self.get_message(self.linear_velocity, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)
    self.get_logger().info('Position: "%s"' % self.current_position())

  def move_backward(self):
    msg = self.get_message(-self.linear_velocity, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)
  
  def mark_visited(self):
    MAP_COVER_RADIUS = int(ROBOT_COVER_RADIUS / map_resolution)
    robot_x, robot_y = self.real_to_map_position(self.pos_to_tuple(self.current_position()))
    for x in range (robot_x - MAP_COVER_RADIUS, robot_x + MAP_COVER_RADIUS + 1):
      for y in range (robot_y - MAP_COVER_RADIUS, robot_y + MAP_COVER_RADIUS + 1):
        if not self.is_out_of_bound((x, y)):
          visited[x, y] = True
  
  # DEBUG
  def print_visited_map(self):
    with open('visited_map.txt', 'w') as file:
      for y in range(0, map_height):
        for x in range(0, map_width):
          if(visited[x, y]):
            file.write('1')
          else:
            file.write('0')
        file.write('\n')
      file.close()

  # is left side of the robot on the map, not relatively to the robot, unvisited
  def is_left_side_unvisited(self):
    current_pose = self.current_position()
    left_map = self.real_to_map_position(self.pos_to_tuple(current_pose))
    self.get_logger().info('Map position "%s "%s"' %(left_map[0], left_map[1]))
    left_map = (left_map[0], int(left_map[1] - ROBOT_COMFORT_RADIUS * map_resolution))
    self.get_logger().info('Current position "%s" "%s"' %(current_pose.x, current_pose.y))
    self.get_logger().info('Map position left "%s "%s"' %(left_map[0], left_map[1]))
    return not self.is_out_of_bound and not visited[left_map[0], left_map[1] - 1]

  def is_facing_left(self):
    return self.equal_floats(math.pi/2, self.current_orientation()[2], self.angular_precision)

  def turn(self):
    msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, self.angular_velocity)
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)
  
  # def face_right(self):
  #   return None
  
  # def face_ahead(self):
  #   return None

  
  # # is right side of the robot on the map, not relatively to the robot, unvisited
  # def is_right_side_unvisited(self):
  #   return False
  
  # def is_facing_right(self):
  #   return None
  
    
  def move_left_unvisited(self):
    if not self.is_facing_left():
      self.turn()
    else:
      self.freeze()
    # elif not self.blocked_ahead():
    #   self.move_ahead()
    # else:
    #   self.face_ahead()

  def get_real_position_ahead(self, pos, distance, theta):
    new_x = pos[0] + math.cos(theta) * distance
    new_y = pos[1] + math.sin(theta) * distance
    return (new_x, new_y)
  
  def get_real_position_left(self, pos, distance, theta):
    return self.get_position_ahead(pos, distance, theta + math.pi/2)
  
  def get_real_position_right(self, pos, distance, theta):
    return self.get_position_ahead(pos, distance, theta - math.pi/2)

  # pos is expected to be sorted
  def inspected_point_in_map(self, me, pos):
    left_most = min(pos[2][0], pos[1][0])
    right_most = max(pos[2][0], pos[1][0])
    y = me[1]
    x = me[0]
    if x < left_most or x > right_most:
      return False
    
    # d1
    if pos[0][1] <= y and y <= pos[1][1]:
      p0p1= self.get_map_angle(pos[0], pos[1])
      p0p2 = self.get_map_angle(pos[0], pos[2])
      p0x1 = self.get_map_angle(pos[0], (x, y))
      minp = min(p0p1, p0p2)
      maxp = max(p0p1, p0p2)
      if x == 27 and y == 25:
        print(f'Pos 0 is: "%s" "%s"' %(pos[0][0], pos[0][1]))
        print(f'Pos 1 is: "%s" "%s"' %(pos[1][0], pos[1][1]))
        print(f'Pos 2 is: "%s" "%s"' %(pos[2][0], pos[2][1]))
        print(p0p1)
        print(p0p2)
        print(p0x1)
      return self.compare_floats(p0x1, minp, self.angular_precision, 1) and self.compare_floats(p0x1, maxp, self.angular_precision, -1)
    
    # d2
    if pos[1][1] <= y and y <= pos[2][1]:
      p2p0 = self.get_map_angle(pos[2], pos[0])
      p1p3 = self.get_map_angle(pos[1], pos[3])
      p1x2 = self.get_map_angle(pos[1], (x, y))
      p2x2 = self.get_map_angle(pos[2], (x, y))
      # p2 left, p1 right
      if pos[2][0] < pos[1][0]:
        cmp1 = self.compare_floats(p1x2, -math.pi, self.angular_precision, 1) and self.compare_floats(p1x2, p1p3, self.angular_precision, -1)
        cmp2 = self.compare_floats(p2x2, 0, self.angular_precision, 1) and self.compare_floats(p2x2, p2p0, self.angular_precision, -1)    
      # p1 left, p2 right
      else:
        p2x2 = self.turn_angle_positive(self.get_map_angle(pos[2], (x, y)))
        cmp1 = self.compare_floats(p1x2, p1p3, self.angular_precision, 1) and self.compare_floats(p1x2, 0.0, self.angular_precision, -1)
        cmp2 = self.compare_floats(p2x2, p2p0, self.angular_precision, 1) and self.compare_floats(p1x2, math.pi, self.angular_precision, -1)
      return cmp1 and cmp2
    
    # d3
    if pos[2][1] <= y and y <= pos[3][1]:
      p3p1= self.get_map_angle(pos[3], pos[1])
      p3p2 = self.get_map_angle(pos[3], pos[2])
      p3x3 = self.get_map_angle(pos[3], (x, y))
      minp = min(p3p1, p3p2)
      maxp = max(p3p1, p3p2)
      return self.compare_floats(p3x3, minp, self.angular_precision, 1) and self.compare_floats(p3x3, maxp, self.angular_precision, -1)

    return False

  def inspected_square_in_map(self, me, pos):
    cnt = 0
    if self.inspected_point_in_map((me[0], me[1]), pos):
      cnt += 1
    if self.inspected_point_in_map((me[0] + 1, me[1]), pos):
      cnt += 1
    if self.inspected_point_in_map((me[0], me[1] + 1), pos):
      cnt += 1
    if self.inspected_point_in_map((me[0] + 1, me[1] + 1), pos):
      cnt += 1
    return cnt >= 2
  
  # DEBUG
  def map_area_blocked_with_printing(self, pos):
    ret = False
    with open("inspecting_map.txt", 'w') as file:
      # sort pos in increasing order of y-coordinate
      for a in range(2, -1, -1):
        for b in range(a + 1, 4):
          if pos[b][1] < pos[b - 1][1] or (pos[b][1] == pos[b - 1][1] and pos[b][0] < pos[b - 1][0]):
            tmp = pos[b]
            pos[b] = pos[b - 1]
            pos[b - 1] = tmp

      # angle check
      for y in range(0, map_height):
        for x in range(0, map_width):
          if self.inspected_point_in_map((x, y), pos):
            if self.blocked((x, y)):
              ret = True
            if x == pos[0][0] and y == pos[0][1]:
              file.write("P")
            elif x == pos[1][0] and y == pos[1][1]:
              file.write("P")
            elif x == pos[2][0] and y == pos[2][1]:
              file.write("P")
            elif x == pos[3][0] and y == pos[3][1]:
              file.write("P")
            else:
              file.write("I")
          else: 
            if x == pos[0][0] and y == pos[0][1]:
              file.write("P")
            elif x == pos[1][0] and y == pos[1][1]:
              file.write("P")
            elif x == pos[2][0] and y == pos[2][1]:
              file.write("P")
            elif x == pos[3][0] and y == pos[3][1]:
              file.write("P")
            else:
              file.write(".")
        file.write("\n")
      
    return ret
      

  def map_area_blocked(self, pos):
    # sort pos in increasing order of y-coordinate
    for a in range(2, -1, -1):
      for b in range(a + 1, 4):
        if pos[b][1] < pos[b - 1][1] or (pos[b][1] == pos[b - 1][1] and pos[b][0] < pos[b - 1][0]):
          tmp = pos[b]
          pos[b] = pos[b - 1]
          pos[b - 1] = tmp

    left_most = min(pos[2].x, pos[1].x)
    right_most = max(pos[2].x, pos[1].x)

    # angle check
    # d1
    for y in range(pos[0].y, pos[1].y + 1):
      for x in range(left_most, right_most + 1):
        p0p1= self.get_map_angle(pos[0], pos[1])
        p0p2 = self.get_map_angle(pos[0], pos[2])
        p0x1 = self.get_map_angle(pos[0], (x, y))
        minp = min(p0p1, p0p2)
        maxp = max(p0p1, p0p2)
        if self.compare_floats(p0x1, minp, self.angular_precision, 1) and self.compare_floats(p0x1, maxp, self.angular_precision, -1):
          if self.blocked(x, y):
            return True
          
    # d2
    for y in range(pos[1].y, pos[2].y + 1):
      for x in range(left_most, right_most + 1):
        p2p0 = self.get_map_angle(pos[2], pos[0])
        p1p3 = self.get_map_angle(pos[1], pos[3])
        p1x2 = self.get_map_angle(pos[1], (x, y))
        p2x2 = self.get_map_angle(pos[2], (x, y))

        # p2 left, p1 right
        if pos[2].x < pos[1].x:
          cmp1 = self.compare_floats(p1x2, -math.pi, self.angular_precision, 1) and self.compare_floats(p1x2, p1p3, self.angular_precision, -1)
          cmp2 = self.compare_floats(p2x2, 0, self.angular_precision, 1) and self.compare_floats(p2x2, p2p0, self.angular_precision, -1)    
        # p1 left, p2 right
        else:
          p2x2 = self.turn_angle_positive(self.get_map_angle(pos[2], (x, y)))
          cmp1 = self.compare_floats(p1x2, p1p3, self.angular_precision, 1) and self.compare_floats(p1x2, 0.0, self.angular_precision, -1)
          cmp2 = self.compare_floats(p2x2, p2p0, self.angular_precision, 1) and self.compare_floats(p1x2, math.pi, self.angular_precision, -1)
        
        if cmp1 and cmp2:
          if self.blocked(x, y):
            return True

    # d3
    for y in range(pos[2].y, pos[3].y + 1):
      for x in range(left_most, right_most + 1):
        p3p1= self.get_map_angle(pos[3], pos[1])
        p3p2 = self.get_map_angle(pos[3], pos[2])
        p3x3 = self.get_map_angle(pos[3], (x, y))
        minp = min(p3p1, p3p2)
        maxp = max(p3p1, p3p2)
        if self.compare_floats(p3x3, minp, self.angular_precision, 1) and self.compare_floats(p3x3, maxp, self.angular_precision, -1):
          if self.blocked(x, y):
            return True
    
    return False

  def blocked(self, pos):
    return image_array[pos[0], pos[1]] == BLOCKED
  
  def turn_angle_positive(self, angle):
    if angle >= 0:
      return angle
    return angle + math.pi * 2
  
  # angle of a vector [-pi, pi)
  def get_map_angle(self, me, other):
    dx = other[0] - me[0]
    dy = me[1] - other[1]
    if dx == 0:
      if dy >= 0:
        return math.pi/2
      return -math.pi/2
    
    if dx > 0:
      angle = math.atan(dy/dx)
    else:
      angle = math.pi + math.atan(dy/dx)

    if self.compare_floats(math.pi, angle, self.angular_precision, -1):
      angle -= 2 * math.pi
    return angle

  def blocked_ahead(self):
    theta = self.current_orientation()[2]
    current_pos = self.pos_to_tuple(self.current_position())
    current_pos_left = self.get_real_position_left(current_pos, ROBOT_COVER_RADIUS, theta)
    current_pos_right = self.get_real_position_right(current_pos, ROBOT_COVER_RADIUS, theta)

    new_pos_left = self.get_real_position_ahead(self, current_pos_left, ROBOT_COMFORT_RADIUS, theta)
    new_pos_right = self.get_real_position_ahead(self, current_pos_right, ROBOT_COMFORT_RADIUS, theta)
    
    current_map_pos_left = self.real_to_map_position(current_pos_left)
    current_map_pos_right = self.real_to_map_position(current_pos_right)
    new_map_pos_left = self.real_to_map_position(new_pos_left)
    new_map_pos_right = self.real_to_map_position(new_pos_right)

    return not self.map_area_blocked([current_map_pos_left, current_map_pos_right, new_map_pos_left, new_map_pos_right])

  def current_orientation(self):
    orientation = self.current_pose.orientation
    return Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz', degrees = False)

  def current_position(self):
    return self.current_pose.position
  
  def real_to_map_position(self, pos):
    x = int((pos[0] - map_origin[0]) / map_resolution)
    y = map_height - int((pos[1] - map_origin[1]) / map_resolution)
    return (x, y)
  
  def pos_to_tuple(self, pos):
    return (pos.x, pos.y)
  
  def move(self):
    return
    self.map_print_inspecting_zone()
    return
    if not self.blocked_ahead():
      self.move_ahead()
    else:
      self.freeze()

    # if self.is_left_side_unvisited():
    #   self.move_left_unvisited()
    # elif not self.blocked_ahead():
    #   self.move_ahead()
    # elif self.is_right_side_unvisited():
    #   self.change_line()
    # else:
    #   self.move_backward()

  # def change_line(self):
  #   self.face_right()
  #   self.move_ahead()
  #   self.face_right()


    
  # def face_right(self):
  #   return None


  # def move_forward():
  #   return None

  # def next_line():
  #   return None

  # def go_to_starting_position():
  #   return None

  # def finish():
  #   return None
    
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


# print(f"Image size: {image.size}")
# print(f"Image mode: {image.mode}")
# print("Pixel data:")
# print(image_array)

# get current robot's position

# robot goes to initial pose (corner of the room)

# robot goes back and forth in a line



# robot moves to next line