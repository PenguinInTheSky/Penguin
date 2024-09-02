import math
from params import *

def get_real_position_ahead(pos, distance, theta):
  print(math.cos(theta))
  print(math.sin(theta))
  new_x = pos[0] + math.cos(theta) * distance
  new_y = pos[1] + math.sin(theta) * distance
  return (new_x, new_y)

def get_real_position_left(pos, distance, theta):
  return get_real_position_ahead(pos, distance, theta + math.pi/2)

def get_real_position_right(pos, distance, theta):
  return get_real_position_ahead(pos, distance, theta - math.pi/2)

def pythagoras_distance(me, other):
  dx = me[0] - other[0]
  dy = me[1] - other[1]
  return math.sqrt(dx * dx + dy * dy)


def make_angle_positive(angle):
  if angle >= 0:
    return angle
  return angle + math.pi * 2

def pos_to_tuple(pos):
  return (pos.x, pos.y)


# return true if 2 floats are equal to a certain precision
def equal_floats(float_main, float_other, precision):
  if (float_main == 0.0):
    return abs(float_other) <= precision * 2
  return abs((float_other - float_main) / float_main) <= precision


# cmp_code: 0 = equal, 1 = main >= other, 2 = main > other, -1 = main <= other, -2 = main < other
def compare_floats(float_main, float_other, precision, cmp_code):
  equal = equal_floats(float_main, float_other, precision)
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


  
# angle of a vector [-pi, pi), soth: -pi/2, north: pi/2, west: -pi, east: 0
def get_map_angle(me, other, angular_precision):
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

  if compare_floats(math.pi, angle, angular_precision, -1):
    angle -= 2 * math.pi
  return angle