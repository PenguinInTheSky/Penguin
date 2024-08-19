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

  def move_ahead(self):
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = 0.0
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)

  def change_line(self):
    msg = Twist()
    msg.angular.z = math.pi / 2
    self.publisher_.publish(msg)
    msg = Twist()
    msg.linear.y = 1.0
    self.publisher_.publish(msg)
    msg = Twist()
    msg.angular.z = math.pi / 2
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)


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