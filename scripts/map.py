from utils import *
from params import *
import yaml
from PIL import Image
import numpy
import os
from ament_index_python.packages import get_package_share_directory
import math


class Map():
  def __init__(self):
    # Load the map image and YAML file to initialize the map object
    pkg_path = os.path.join(get_package_share_directory('Penguin'))
    pgm_file = os.path.join(pkg_path, 'maps', 'small_room', 'small_room_saved.pgm')

    self.image = Image.open(pgm_file)

    yaml_file = os.path.join(pkg_path, 'maps', 'small_room', 'small_room_saved.yaml')
    with open(yaml_file, 'r') as file:
      map_info = yaml.safe_load(file)

    self.width, self.height = self.image.size

    self.origin = map_info['origin']
    self.resolution = map_info['resolution']

    # Initialize a 2D numpy array to track visited squares in the map
    self.visited = numpy.zeros([self.width, self.height], dtype = bool)

    # Add wall thickness for safety reasons to avoid the robot getting too close to walls
    self.wall_thickness = 4

    for x in range(0, self.width):
      for y in range(0, self.height):
        pixel = self.image.getpixel((x, y))
        if pixel != EMPTY or self.wall((x, y)):
          self.visited[x, y] = True

  # mark a square as visited based on the robot's position and the robot's cover radius for safety reason
  def mark_visited(self, pos):
    MAP_COVER_RADIUS = int(ROBOT_COVER_RADIUS / self.resolution)
    robot_x, robot_y = self.real_to_map_position(pos_to_tuple(pos))
    for x in range (robot_x - MAP_COVER_RADIUS, robot_x + MAP_COVER_RADIUS + 1):
      for y in range (robot_y - MAP_COVER_RADIUS, robot_y + MAP_COVER_RADIUS + 1):
        distance = pythagoras_distance((robot_x, robot_y), (x, y))
        if not self.is_out_of_bound((x, y)) and compare_floats(MAP_COVER_RADIUS, distance, LINEAR_PRECISION, 1):
          self.visited[x, y] = True

  # DEBUGGING FUNCTION
  def print_visited_map(self):
    with open('visited_map.txt', 'w') as file:
      for x in range(self.width - 1, -1, -1):
        for y in range(0, self.height):
          if(self.visited[x, y]):
            file.write('1')
          else:
            file.write('0')
        file.write('\n')
      file.close()

  # Convert robot pose to map coordinates based on the map's origin and resolution
  def real_to_map_position(self, pos):
    x = int((pos[0] - self.origin[0]) / self.resolution)
    y = self.height - int((pos[1] - self.origin[1]) / self.resolution)
    return (x, y)
  
  # Check if a given map position is out of bounds of the map
  def is_out_of_bound(self, pos):
    return pos[0] < 0 or pos[1] < 0 or pos[0] >= self.width or pos[1] >= self.height

  # Check if a given map position has been visited
  def is_visited(self, x, y):
    return self.visited[x, y]

  # pos is expected to be sorted in increasing order of y-coordinate
  def inspected_point_in_map(self, me, pos):
    left_most = min(pos[2][0], pos[1][0])
    right_most = max(pos[2][0], pos[1][0])
    y = me[1]
    x = me[0]
    if x < left_most or x > right_most:
      return False
    
    # consider different cases for different shapes created by the four corners in pos
    
    # d1 
    if pos[0][1] <= y and y <= pos[1][1]:
      p0p1= get_map_angle(pos[0], pos[1], ANGULAR_PRECISION)
      p0p2 = get_map_angle(pos[0], pos[2], ANGULAR_PRECISION)
      p0x1 = get_map_angle(pos[0], (x, y), ANGULAR_PRECISION)
      minp = min(p0p1, p0p2)
      maxp = max(p0p1, p0p2)
      return compare_floats(p0x1, minp, ANGULAR_PRECISION, 1) and compare_floats(p0x1, maxp, ANGULAR_PRECISION, -1)
    
    # d2
    if pos[1][1] <= y and y <= pos[2][1]:
      p2p0 = get_map_angle(pos[2], pos[0], ANGULAR_PRECISION)
      p1p3 = get_map_angle(pos[1], pos[3], ANGULAR_PRECISION)
      p1x2 = get_map_angle(pos[1], (x, y), ANGULAR_PRECISION)
      p2x2 = get_map_angle(pos[2], (x, y), ANGULAR_PRECISION)
      # p2 left, p1 right
      if pos[2][0] < pos[1][0]:
        cmp1 = compare_floats(p1x2, -math.pi, ANGULAR_PRECISION, 1) and compare_floats(p1x2, p1p3, ANGULAR_PRECISION, -1)
        cmp2 = compare_floats(p2x2, 0, ANGULAR_PRECISION, 1) and compare_floats(p2x2, p2p0, ANGULAR_PRECISION, -1)    
      # p1 left, p2 right
      else:
        p2x2 = get_map_angle(pos[2], (x, y)), ANGULAR_PRECISION
        cmp1 = compare_floats(p1x2, p1p3, ANGULAR_PRECISION, 1) and compare_floats(p1x2, 0.0, ANGULAR_PRECISION, -1)
        cmp2 = compare_floats(p2x2, p2p0, ANGULAR_PRECISION, 1) and compare_floats(p1x2, math.pi, ANGULAR_PRECISION, -1)
      return cmp1 and cmp2
    
    # d3
    if pos[2][1] <= y and y <= pos[3][1]:
      p3p1= get_map_angle(pos[3], pos[1], ANGULAR_PRECISION)
      p3p2 = get_map_angle(pos[3], pos[2], ANGULAR_PRECISION)
      p3x3 = get_map_angle(pos[3], (x, y), ANGULAR_PRECISION)
      minp = min(p3p1, p3p2)
      maxp = max(p3p1, p3p2)
      return compare_floats(p3x3, minp, ANGULAR_PRECISION, 1) and compare_floats(p3x3, maxp, ANGULAR_PRECISION, -1)

    return False

  # Check if the unit square starting from "me" in the map lies within the area defined by the four corners in "pos"
  # It is considered to be within the area if at least 2 of the 4 corners of the unit square are within the area defined by "pos"
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
  
  def map_area_blocked_with_printing(self, pos, robot_pos):
    ret = False
    with open("inspecting_map.txt", 'w') as file:
      # bubble sort pos in increasing order of y-coordinate, then x-coordinate
      for a in range(2, -1, -1):
        for b in range(a + 1, 4):
          if pos[b][1] < pos[b - 1][1] or (pos[b][1] == pos[b - 1][1] and pos[b][0] < pos[b - 1][0]):
            tmp = pos[b]
            pos[b] = pos[b - 1]
            pos[b - 1] = tmp

      current_map = self.real_to_map_position(pos_to_tuple(robot_pos))
      
      # angle check
      for x in range(self.width - 1, -1, -1):
        for y in range(0, self.height):
          # Check if the square at (x, y) is within the area being inspected and if it is blocked 
          # (loosen the strict requirements for blocked squares to be considered as blocked, instead of just the point x, y)
          if self.inspected_square_in_map((x, y), pos):
            if self.blocked((x, y)):
              ret = True
              
          # robot's current position is marked with "O" in the printed map
          if x == current_map[0] and y == current_map[1]:
            file.write("O")
            continue
          # the four corners of the area being inspected are marked with "P" in the printed map
          if x == pos[0][0] and y == pos[0][1]:
            file.write("P")
          elif x == pos[1][0] and y == pos[1][1]:
            file.write("P")
          elif x == pos[2][0] and y == pos[2][1]:
            file.write("P")
          elif x == pos[3][0] and y == pos[3][1]:
            file.write("P")
          # blocked squares are marked with "X" in the printed map
          elif self.blocked((x, y)):
            file.write("X")
          # unblocked squares are marked with "." in the printed map
          else:
            file.write(".")
        file.write("\n")
      
    return ret
  
  # Check if a given position is blocked based on the map's boundaries, walls, and blocked pixels
  def blocked(self, pos):
    return self.is_out_of_bound(pos) or self.wall(pos) or self.image.getpixel((pos[0], pos[1])) == BLOCKED

  # Check if a square in a specific direction (angle) needs to be visited based on the current faux position and map information
  def blocked_ahead_angle(self, theta, robot_pos):
    current_pos = pos_to_tuple(robot_pos)
    
    # calculate the far left and right of robot's current position's covering area
    current_pos_left = get_real_position_left(current_pos, ROBOT_COVER_RADIUS, theta)
    current_pos_right = get_real_position_right(current_pos, ROBOT_COVER_RADIUS, theta)

    # calculate the far left and right of robot's next position's comfort zone area
    new_pos_left = get_real_position_ahead(current_pos_left, ROBOT_COMFORT_RADIUS, theta)
    new_pos_right = get_real_position_ahead(current_pos_right, ROBOT_COMFORT_RADIUS, theta)
    
    # convert the position to map coordinates for checking if the area is blocked
    current_map_pos_left = self.real_to_map_position(current_pos_left)
    current_map_pos_right = self.real_to_map_position(current_pos_right)
    new_map_pos_left = self.real_to_map_position(new_pos_left)
    new_map_pos_right = self.real_to_map_position(new_pos_right)

    # check the area formed by the above four points are not blocked by walls or obstacles in the map
    return self.map_area_blocked_with_printing([current_map_pos_left, current_map_pos_right, new_map_pos_left, new_map_pos_right], robot_pos)
  
  # Check if a given position is a wall based on the map's wall thickness and boundaries
  def wall(self, pos):
    return pos[0] < self.wall_thickness or pos[0] >= self.width - self.wall_thickness or pos[1] < self.wall_thickness or pos[1] >= self.height - self.wall_thickness