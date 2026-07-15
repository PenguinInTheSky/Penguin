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

# Publish the initial pose of the robot to the 'initialpose' topic
class InitialPosePublisher(Node):
  def __init__(self):
    super().__init__('initial_pose_publisher')
    # Create a publisher for the 'initialpose' topic with a queue size of 1
    self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
    
    # Set the initial pose of the robot
    initial_pose = PoseWithCovarianceStamped()
    
    # Set the header information
    initial_pose.header.stamp = self.get_clock().now().to_msg()
    initial_pose.header.frame_id = 'map'

    # Set the initial position (x, y, z)
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
 
# Control robot movement based on the current pose and map information
class RobotDriver(Node):
  def __init__(self):
    super().__init__('robot_driver')

    # Subscribe to the '/amcl_pose' topic to receive the robot's current pose
    self.subscription = self.create_subscription(
      PoseWithCovarianceStamped,
      '/amcl_pose',
      self.pose_callback,
      10
    )

    # Create a publisher for the 'diff_cont/cmd_vel_unstamped' topic to send velocity commands
    self.publisher_ = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
    
    # Create a timer to call the move function at a specified rate
    self.timer = self.create_timer(PUBLISH_RATE, self.move)
    
    # Initialize the robot's current pose and other state variables
    self.current_pose = Pose()
    
    # Initialize the robot's current faux position being the same as the current pose position
    # Current faux position is used to track the robot's position in the map 
    # before the robot starts turning in different directions for exploration to analyse next move
    self.current_faux_position = self.current_pose.position
    
    # Initialize the robot's frozen and turning states
    self.frozen = False
    self.turning = False
    
    # Initialize the map object to track visited squares and blocked paths
    self.map = Map()
    
    self.log("Created robot driver", 0)

  # Log a message, optionally gated behind DEBUG log level
  def log(self, msg, debug):
    if debug:
      if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
        self.get_logger().info(msg)
    else:
      self.get_logger().info(msg)

  def pose_callback(self, msg):
    self.log('Received pose: "%s"' % msg.pose.pose, 0)
    self.current_pose = msg.pose.pose
    if not self.turning:
      self.current_faux_position = self.current_pose.position
    self.map.mark_visited(self.current_position())
    self.map.print_visited_map()
    # self.log('Robot blocked ahead is %s"' %self.blocked_ahead(), 1)
    # # self.get_logger().info('Left_side_unvisited is "%s' % self.is_left_side_unvisited())
    # self.log('Current posing is: "%s"' % self.current_position(), 0)
    # self.log('Current orientation is: "%s"' % self.current_orientation(), 0)

  # Create a Twist message with the specified linear and angular velocities
  def get_message(self, x, y, z, roll, pitch, yaw):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.x = roll
    msg.angular.y = pitch
    msg.angular.z = yaw
    return msg

  # Get the current orientation of the robot in Euler angles (roll, pitch, yaw)
  def current_orientation(self):
    orientation = self.current_pose.orientation
    return Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz', degrees = False)

  # Get the current position of the robot
  def current_position(self):
    return self.current_pose.position
  
  # Check if a square in a specific direction (angle) needs to be visited based on the current faux position and map information
  def does_this_square_angle_need_visiting(self, square_angle):
    current_pose = self.current_faux_position
    
    # Check if the path ahead is blocked for the given angle
    if self.map.blocked_ahead_angle(square_angle, current_pose):
      return False
    
    # Calculate the next position ahead in map coordinates based on the current pose and the specified angle
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
      
    # Check if the next position is within bounds and has not been visited yet
    return not self.map.is_out_of_bound((next_map[0], next_map[1])) and not self.map.is_visited(next_map[0], next_map[1])

  # Check if the robot is facing a specific angle within a certain accuracy rate
  def is_facing_this_angle(self, angle):
    current_angle = self.current_orientation()[2]
    self.log("Current angle: %s, Target angle: %s" % (current_angle, angle), 0)
    cmp1 = equal_floats(angle, current_angle, ANGULAR_PRECISION)
    cmp2 = equal_floats(angle, current_angle - 2*math.pi, ANGULAR_PRECISION)
    return cmp1 or cmp2

  # Freeze the robot by publishing a zero velocity message
  def freeze(self):
    msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.frozen = True
    self.log('Freeze robot', 1)

  # Turn the robot to face a specific angle by publishing an angular velocity message
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
  
  # Stop the robot's turning motion by publishing a zero velocity message
  def stop_turn(self):
    msg = self.get_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.log('Publishing: "%s"' % msg, 1)
    self.turning = False
    
  # Move the robot ahead in the direction it is currently facing
  def move_ahead(self):
    msg = self.get_message(LINEAR_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.publisher_.publish(msg)
    self.log('Publishing: "%s"' % msg, 1)
    self.log('Position: "%s"' % self.current_position(), 1)

  # Move the robot in a specific direction based on the angle provided, 
  # if the path is not blocked and the square has not been visited yet
  def move_squared_angle_unvisited(self, angle):
    # Try to face the angle that the robot needs to face before moving
    self.log("Trying to face angle: %s" % angle, 0)
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
      
    # Move ahead if the path is not blocked
    elif not self.map.blocked_ahead_angle(angle, self.current_faux_position):
      self.stop_turn()
      if angle == math.pi/2:
        self.log("Moving left", 0)
      elif angle == 0.0:
        self.log("Moving north", 0)
      elif angle == -math.pi:
        self.log("Moving south", 0)
      else:
        self.log("Moving right", 0)
      self.move_ahead()

  # Move the robot based on the availability of unvisited squares in the four cardinal directions, 
  # if none are available, move in any unblocked direction, if none are available, freeze the robot
  def move(self):
    if self.does_this_square_angle_need_visiting(math.pi/2):
      # Move left
      self.move_squared_angle_unvisited(math.pi/2)
    elif self.does_this_square_angle_need_visiting(0.0):
      # Move ahead
      self.move_squared_angle_unvisited(0.0)
    elif self.does_this_square_angle_need_visiting(-math.pi):
      # Move behind
      self.move_squared_angle_unvisited(-math.pi)
    elif self.does_this_square_angle_need_visiting(-math.pi/2):
      # Move right
      self.move_squared_angle_unvisited(-math.pi/2)
    
    # Fall-back to unblocked directions if no unvisited squares are available to recover from dead-ends
    elif not self.map.blocked_ahead_angle(math.pi/2, self.current_faux_position):
      # Move left
      self.move_squared_angle_unvisited(math.pi/2)
    elif not self.map.blocked_ahead_angle(0.0, self.current_faux_position):
      # Move ahead
      self.move_squared_angle_unvisited(0.0)
    elif not self.map.blocked_ahead_angle(-math.pi, self.current_faux_position):
      # Move behind
      self.move_squared_angle_unvisited(-math.pi)
    elif not self.map.blocked_ahead_angle(-math.pi/2, self.current_faux_position):
      # Move right
      self.move_squared_angle_unvisited(-math.pi/2)
    
    # Freeze if there is no where else to go
    else:
      self.freeze()

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
