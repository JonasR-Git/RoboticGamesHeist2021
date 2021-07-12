import math

ROBOTER_SIZE = 15  # in cm
MAP_SIZE = 1000
HALF_ROBOTER_SQUARED_SIZE = (ROBOTER_SIZE / 2) ** 2
GRID_BLOCK_SIZE = 5
MAP_TILES_PER_AXIS = int(MAP_SIZE / GRID_BLOCK_SIZE)
DEFAULT_FORWARD = (1, 0)
ROBOTER_ORIENTATION = 0
ROBOTER_POSITION_X = 0
ROBOTER_POSITION_Y = 0


def is_tile_in_bounds(x, y):
    return 0 <= x < MAP_TILES_PER_AXIS and 0 <= y < MAP_TILES_PER_AXIS


def get_radians_from_orientation(orientation):
    return orientation  # * 2 * math.pi


def local_to_global_orientation(local_orientation):
    return ROBOTER_ORIENTATION + local_orientation


def get_direction_from_orientation(orientation):
    radians = get_radians_from_orientation(orientation)
    return (
        # forward direction x value
        DEFAULT_FORWARD[0] * math.cos(radians) - DEFAULT_FORWARD[1] * math.sin(radians),
        # forward direction y value
        DEFAULT_FORWARD[0] * math.sin(radians) + DEFAULT_FORWARD[1] * math.cos(radians))
