from PIL import Image
import numpy
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation
import math

UNKNOWN = 205
EMPTY = 254
OCCUPIED = 0


class AmclPoseListener(Node):
  def __init__(self):
    super().__init__('amcl_pose_listener')
    self.subscription = self.create_subscription(
      PoseWithCovarianceStamped,
      '/amcl_pose',
      self.pose_callback,
      10
    )
  def pose_callback(self, msg):
    position = msg.pose.pose.position
    self.get_logger().info(f'Position: x={position.x}, y={position.y}, z={position.z}')

    orientation = msg.pose.pose.orientation
    rotation = Rotation.from_quat(orientation)
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
    
    self.get_logger().info(f'Orientation: roll={roll}, pitch={pitch}, yaw={yaw}')

class RobotDriver(Node):
  def __init__(self):
    super().__init__('robot_driver')
    self.publisher_ = self.create_publisher(Twist, 'tri_cont/cmd_vel', 10)
    self.timer = self.create_timer(0.1, self.move)

  # is left side of the robot on the map, not relatively to the robot, unvisited
  def is_left_side_unvisited(self):
    return False

  def is_facing_left(self):
    return None
  

  # is right side of the robot on the map, not relatively to the robot, unvisited
  def is_right_side_unvisited(self):
    return False
  
  def is_facing_right(self):
    return None
  

  def face_left(self):
    return None
  
  def face_right(self):
    return None
  
  def face_ahead(self):
    return None
    
  def move_left_unvisited(self):
    if not self.is_facing_left():
      self.face_left()
    if not self.blocked_ahead():
      self.move_ahead()
    else:
      self.face_ahead()

  def move_ahead(self):
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = 0.0
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)  
  
  def change_line(self):
    self.face_right()
    self.move_ahead()
    self.face_right()
    
  def move_backward():
    return None


  def move(self):
    if self.is_left_side_unvisited():
      self.move_left_unvisited()
    elif not self.blocked_ahead():
      self.move_ahead()
    elif self.is_right_side_unvisited():
      self.change_line()
    else:
      self.move_backward()
    return None
  
    
  def face_right(self):
    return None

  def blocked_ahead(distance):
    return None

  def move_forward():
    return None

  def next_line():
    return None

  def go_to_starting_position():
    return None

  def finish():
    return None

  def current_pose():
    return None


# get map
pkg_path = os.path.join(get_package_share_directory('Penguin'))
pgm_file = os.path.join(pkg_path, 'maps', 'small_room', 'small_room_saved.pgm')
image = Image.open(pgm_file)
image_array = numpy.array(image)
print(f"Image size: {image.size}")
print(f"Image mode: {image.mode}")
print("Pixel data:")
print(image_array)

# get current robot's position

# robot goes to initial pose (corner of the room)

# robot goes back and forth in a line



# robot moves to next line