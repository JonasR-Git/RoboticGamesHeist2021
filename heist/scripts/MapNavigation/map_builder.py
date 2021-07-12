import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from ..helpers import (MAP_TILES_PER_AXIS, get_direction_from_orientation, ROBOTER_SIZE, GRID_BLOCK_SIZE,
                       local_to_global_orientation, ROBOTER_POSITION_X, ROBOTER_POSITION_Y, HALF_ROBOTER_SQUARED_SIZE,
                       is_tile_in_bounds)


class MapBuilder:
    __slots__ = ['position', "map", "orientation"]

    def __init__(self):
        self.map = np.ones((MAP_TILES_PER_AXIS, MAP_TILES_PER_AXIS), dtype=bool)

        rospy.Subscriber("scan", LaserScan, self.scan_callback)

    def add_point_to_map(self, obstacle_distance, obstacle_local_orientation):
        global_orientation = local_to_global_orientation(obstacle_local_orientation)
        obstacle_direction = get_direction_from_orientation(global_orientation)
        obstacle_x = int(round(ROBOTER_POSITION_X + obstacle_direction[0] * obstacle_distance))
        obstacle_y = int(round(ROBOTER_POSITION_Y + obstacle_direction[1] * obstacle_distance))

        # mark all spots as not reachable in a circle of half the roboter
        # square distance as radius as not reachable

        # maybe only do this when the field is marked as reachable
        # if(not map[obstacleX][obstacleY]):
        offset_size = int(ROBOTER_SIZE / (GRID_BLOCK_SIZE * 2))
        for x in range(-offset_size, offset_size + 1, 1):
            for y in range(-offset_size, offset_size + 1, 1):
                # check if distance from obstacle point is within squared roboter radius
                # maybe round here
                if ((x * x + y * y) / GRID_BLOCK_SIZE < HALF_ROBOTER_SQUARED_SIZE
                        and is_tile_in_bounds(x + obstacle_x, y + obstacle_y)):
                    self.map[x + obstacle_x, y + obstacle_y] = False

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        radians_increment = msg.angle_increment
        current_radians = msg.angle_min

        for r in ranges:
            if not math.isfinite(r):
                self.add_point_to_map(r, current_radians)
            current_radians += radians_increment
