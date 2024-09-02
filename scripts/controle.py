#!/usr/bin/env python3
from utils import *
from params import *
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from scipy.spatial.transform import Rotation
import math
from map import Map

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
    self.publish_rate = 0.2
    self.timer = self.create_timer(self.publish_rate, self.move)
    self.current_pose = Pose()
    self.frozen = False
    self.turning = False
    self.map = Map()
    self.log("Created robot driver", 0)

  def pose_callback(self, msg):
    if not self.turning:
      self.current_pose.position = msg.pose.pose.position
    self.current_pose.orientation = msg.pose.pose.orientation
    self.map.mark_visited(self.current_position())
    self.map.print_visited_map()
    # self.log('Robot blocked ahead is %s"' %self.blocked_ahead(), 1)
    # # self.get_logger().info('Left_side_unvisited is "%s' % self.is_left_side_unvisited())
    # self.log('Current posing is: "%s"' % self.current_position(), 0)
    # self.log('Current orientation is: "%s"' % self.current_orientation(), 0)

  def get_message(self, x, y, z, roll, pitch, yaw):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.x = roll
    msg.angular.y = pitch
    msg.angular.z = yaw
    return msg
  


  # DEBUG
  def print_pose(self):
    self.log('Current posing is: "%s"' % self.current_position(), 0)
    self.log('Current orientation is: "%s"' % self.current_orientation(), 0)


  # DEBUG
  def log(self, msg, debug):
    if debug:
      if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
        self.get_logger().info(msg)
    else:
      self.get_logger().info(msg)


  # ROBOT CURRENT STATE, SUBSCRIBER ROLE

  def current_orientation(self):
    orientation = self.current_pose.orientation
    return Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz', degrees = False)

  def current_position(self):
    return self.current_pose.position
  
  def does_this_square_angle_need_visiting(self, square_angle):
    current_pose = self.current_position()
    if self.map.blocked_ahead_angle(square_angle, current_pose):
      if square_angle == math.pi/2:
        self.log("Left is blocked", 0)
      elif square_angle == 0.0:
        self.log("North is blocked", 0)
      elif square_angle == -math.pi:
        self.log("South is blocked", 0)
      else:
        self.log("Right is blocked", 0)
      return False
    next_pose = get_real_position_ahead(pos_to_tuple(current_pose), ROBOT_COMFORT_RADIUS, square_angle)
    next_map = self.map.real_to_map_position(next_pose)
    if square_angle == 0.0:
      next_map = (next_map[0] + 1, next_map[1])
    elif square_angle == -math.pi:
      next_map = (next_map[0] - 1, next_map[1])
    elif square_angle == -math.pi/2:
      next_map = (next_map[0], next_map[1] + 1)
    else:
      next_map = (next_map[0], next_map[1] - 1)
    # self.log('Current pose: "%s"' %current_pose, 0)
    # self.log('Next pose: "%s" "%s"' %(next_pose[0], next_pose[1]), 0)
    # # self.get_logger().info('Map position "%s "%s"' %(left_map[0], left_map[1]))
    current_map = self.map.real_to_map_position(pos_to_tuple(current_pose))
    # self.log('Current pose in map: "%s" "%s"' %(current_map[0], current_map[1]), 0)
    # self.log('Next pose in map: "%s" "%s"' %(next_map[0], next_map[1]), 0)
    # self.get_logger().info('Current position "%s" "%s"' %(current_pose.x, current_pose.y))
    # self.get_logger().info('Map position left "%s "%s"' %(left_map[0], left_map[1]))
    # if not (not self.is_out_of_bound((next_map[0], next_map[1])) and not visited[next_map[0], next_map[1]]):
    #   if square_angle == math.pi/2:
    #     self.log("Left visited", 0)
    #   elif square_angle == 0.0:
    #     self.log("North visited", 0)
    #   elif square_angle == -math.pi:
    #     self.log("South visited", 0)
    #   else:
    #     self.log("Right visited", 0)
    return not self.map.is_out_of_bound((next_map[0], next_map[1])) and not self.map.is_visited(next_map[0], next_map[1])

  # is left side of the robot on the map, not relatively to the robot, unvisited
  def is_facing_this_angle(self, angle):
    current_angle = self.current_orientation()[2]
    cmp1 = equal_floats(angle, current_angle, ANGULAR_PRECISION)
    cmp2 = equal_floats(angle, current_angle - 2*math.pi, ANGULAR_PRECISION)
    return cmp1 or cmp2
 
  # ROBOT OPERATIONS, PUBLISHER ROLE

  def freeze(self):
    msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.frozen = True
    self.log('Freeze robot', 1)

  def move_backward(self):
    msg = self.get_message(-LINEAR_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.log('Publishing: "%s"' % msg, 1)
  
  def turn(self, angle):
    current_angle = self.current_orientation()[2]
    if current_angle > angle:
      angle += 2*math.pi
    if angle - current_angle < math.pi:
      msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, ANGULAR_VELOCITY)
    else:
      msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, -ANGULAR_VELOCITY)
      
    self.turning = True
    self.publisher_.publish(msg)
    self.log('Publishing: "%s"' % msg, 1)
      
  def stop_turn(self):
    msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.log('Publishing: "%s"' % msg, 1)
    self.turning = False
    
  def move_ahead(self):
    msg = self.get_message(LINEAR_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.log('Publishing: "%s"' % msg, 1)
    self.log('Position: "%s"' % self.current_position(), 1)
  
  def change_line(self):
    self.log("changing line", 0)
    if not self.is_facing_this_angle(-math.pi/2):
      # self.log("Trying to face right", 0)
      self.turn(-math.pi/2)
    else:
      self.stop_turn()
      self.move_ahead()
      time.sleep(0.06)
      self.freeze()

  def move_squared_angle_unvisited(self, angle):
    if not self.is_facing_this_angle(angle):
      if angle == math.pi/2:
        self.log("Trying to face left", 0)
      elif angle == 0.0:
        self.log("Trying to face north", 0)
      elif angle == -math.pi:
        self.log("Trying to face south", 0)
      else:
        self.log("Trying to face right", 0)
      self.turn(angle)
    elif not self.map.blocked_ahead_angle(angle, self.current_position()):
      self.stop_turn()
      self.move_ahead()

  def move(self):
    if self.does_this_square_angle_need_visiting(math.pi/2):
      # self.log("Left side unvisited", 0)
      self.move_squared_angle_unvisited(math.pi/2)
    elif self.does_this_square_angle_need_visiting(0.0):
      # self.log("North side unvisited", 0)
      self.move_squared_angle_unvisited(0.0)
    elif self.does_this_square_angle_need_visiting(-math.pi):
      self.move_squared_angle_unvisited(-math.pi)
    elif not self.is_facing_this_angle(-math.pi/2) or not self.map.blocked_ahead_angle(-math.pi/2, self.current_position()):
      self.change_line()
    elif not self.map.blocked_ahead_angle(math.pi/2, self.current_position()):
      self.move_squared_angle_unvisited(math.pi/2)
    elif not self.map.blocked_ahead_angle(0.0, self.current_position()):
      self.move_squared_angle_unvisited(0.0)
    elif not self.map.blocked_ahead_angle(-math.pi, self.current_position()):
      self.move_squared_angle_unvisited(-math.pi)
    elif not self.map.blocked_ahead_angle(-math.pi/2, self.current_position()):
      self.move_squared_angle_unvisited(-math.pi/2)
    else:
      self.freeze()

  # def go_to_starting_position():
  #   return None

  # def finish():
  #   return None


def pretty_print_position(pos):
  print("x: %s" %pos[0])
  print("y: %s" %pos[1])
    
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
