#!/usr/bin/env python3.8
import numpy as np
import math
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from time import perf_counter
import random

MAX_SPEED = 0.4


class StartAreasModel:

    def __init__(self):
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/evader/odom", Odometry, self.listener_callback)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        # occupancy grid data, grid config of occupancy grid
        self.occupancy_grid = []
        self.map_resolution = 0
        self.map_width = 0
        self.map_height = 0
        self.map_width_in_meter = 0
        self.map_height_in_meter = 0

        # noise error data and config
        self.max_error_distance = 1
        self.max_noise_tile_error = 0
        self.sqr_max_noise_tile_error = 0
        self.noise_diameter = 0
        # a tile is considered free if it is under this value
        self.is_reachable_threshold = 5
        self.sqrtOf2 = math.sqrt(2)

        # if a start area has the likelihood of under this value it is not considered an option
        self.threshold_to_reject_start_area = 0.0025

        self.total_reachable_tiles_in_start_area = 0
        self.disconnected_possible_start_areas = 0
        self.node_block_id = np.array((0, 0))
        # value at index i is number of tiles in start area i
        self.nodes_in_block = []
        # list of a 2d array
        # each entry i stores a grid containing the distance from every node to the starting area i
        self.start_area_distances = []
        self.start_area_probabilities = []
        self.most_likely_start_area_number = 0

        self.probability_update_iteration = 1
        self.start_time = perf_counter()
        self.global_open_nodes = []
        self.enemy_approximate_positions = []

        self.has_map = False
        self.initial = True
        self.default_orientation = None
        self.frame_id = 0
        self.last_sent = perf_counter()

    def tester(self, x, y):
        odometry = Odometry()
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        self.listener_callback(odometry)

    def map_callback(self, message):
        if not self.has_map:
            self.occupancy_grid = [x for x in message.data]
            self.map_width = message.info.width
            self.map_height = message.info.height
            self.map_resolution = message.info.resolution
            self.map_width_in_meter = self.map_width * self.map_resolution
            self.map_height_in_meter = self.map_height * self.map_resolution
            self.max_noise_tile_error = math.ceil(self.max_error_distance / self.map_resolution)
            self.sqr_max_noise_tile_error = self.max_noise_tile_error ** 2
            self.noise_diameter = self.max_noise_tile_error * 2 + 1

            # offset occupancy grid so its map aligns with the real world map
            real_world_pos_of_zero_zero = (message.info.origin.position.x, message.info.origin.position.y)

            correct_world_x_pos_half_half = 0
            correct_world_y_pos_half_half = 0

            actual_world_pos_of_half_half_x = real_world_pos_of_zero_zero[0] + self.map_width_in_meter / 2
            actual_world_pos_of_half_half_y = real_world_pos_of_zero_zero[1] + self.map_height_in_meter / 2

            x_diff = actual_world_pos_of_half_half_x - correct_world_x_pos_half_half
            y_diff = actual_world_pos_of_half_half_y - correct_world_y_pos_half_half

            tile_x_movement = int(round(x_diff / self.map_resolution))
            tile_y_movement = int(round(y_diff / self.map_resolution))

            if tile_x_movement != 0 or tile_y_movement != 0:
                # shift occupancy grid based on the origin of the
                for x in range(0, self.map_width - max(0, tile_x_movement)):
                    for y in range(0, self.map_height - max(0, tile_x_movement)):
                        index = self.coord_2_d_to_1_d((x, y))
                        index_two = self.coord_2_d_to_1_d(
                            (x + tile_x_movement, y + tile_y_movement))
                        value = message.data[index]
                        self.occupancy_grid[index_two] = value

            self.has_map = True

            """
            self.tester(0, 0)
            self.tester(-2, 3.0)
            """

    def listener_callback(self, message):
        if self.has_map:
            # the first time a position is received
            if self.initial:
                alpha = 2 * math.pi * random.random()
                x = math.cos(alpha)
                y = math.sin(alpha)
                message.pose.pose.position.x += x
                message.pose.pose.position.x += y
                self.enemy_approximate_positions.append(message)
                self.default_orientation = message.pose.pose.orientation
                self.initial = False
                self.search_separated_possible_start_areas(self.enemy_approximate_positions[0])
                self.calculate_start_area_probabilities()
                self.build_start_area_distances()
            else:
                if perf_counter() - self.last_sent < 2:
                    return
                self.probability_of_point_from_start_block_after_seconds(message,
                                                                         (perf_counter() - self.start_time) * MAX_SPEED)
                print('...')
            halfway_intercept_coordination = self.get_next_guard_position_when_guarding_area(
                self.most_likely_start_area_number)
            halfway_intercept_position = self.index_to_position(halfway_intercept_coordination)
            self.pub.publish(self.get_pose_to_publish(halfway_intercept_position, self.most_likely_start_area_number))
            self.last_sent = perf_counter()

    def is_tile_in_bounds(self, coord):
        return 0 <= coord[0] < self.map_width and 0 <= coord[1] < self.map_height

    def is_start_area_active(self, block_id):
        return self.start_area_probabilities[block_id] >= self.threshold_to_reject_start_area

    def position_to_index(self, x, y):
        x, y = int(round(x / self.map_resolution)), int(round(y / self.map_resolution))
        return x, y

    def get_2d_position_from_odom(self, odom):
        return (
            odom.pose.pose.position.x + self.map_width_in_meter / 2,
            odom.pose.pose.position.y + self.map_height_in_meter / 2)

    def coord_2_d_to_1_d(self, coord):
        return int(coord[1] * self.map_width + coord[0])

    def get_grid_value_at_coord(self, coord):
        return self.occupancy_grid[self.coord_2_d_to_1_d(coord)]

    def position_to_tuple(self, coord):
        return self.position_to_index(coord[0], coord[1])

    def index_to_position(self, coord):
        x, y = coord[0] * self.map_resolution, coord[1] * self.map_resolution
        return x, y

    def get_pose_to_publish(self, p, start_area):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        self.frame_id += 1
        pose.pose.position.x = p[0] - self.map_width_in_meter / 2
        pose.pose.position.y = p[1] - self.map_height_in_meter / 2
        pose.pose.orientation.w = 1
        # pose.pose.position.z = self.start_area_probabilities[start_area]
        return pose

    def is_coord_considered_free(self, coord):
        return 0 <= self.get_grid_value_at_coord(coord) < self.is_reachable_threshold

    def is_coord_considered_free_in_map(self, coord, map_view):
        return 0 <= map_view[coord] < self.is_reachable_threshold

    @staticmethod
    def sqr_magnitude(coord):
        return coord[0] * coord[0] + coord[1] * coord[1]

    @staticmethod
    def get_simple_adjacent_fields(coord):
        return [
            (coord[0] - 1, coord[1]),
            (coord[0], coord[1] - 1),
            (coord[0], coord[1] + 1),
            (coord[0] + 1, coord[1]),
        ]

    def get_adjacent_fields(self, coord):
        return [
            ((coord[0] - 1, coord[1] - 1), self.sqrtOf2 * self.map_resolution),
            ((coord[0] - 1, coord[1]), 1 * self.map_resolution),
            ((coord[0] - 1, coord[1] + 1), self.sqrtOf2 * self.map_resolution),
            ((coord[0], coord[1] - 1), 1 * self.map_resolution),
            ((coord[0], coord[1] + 1), 1 * self.map_resolution),
            ((coord[0] + 1, coord[1] - 1), self.sqrtOf2 * self.map_resolution),
            ((coord[0] + 1, coord[1]), 1 * self.map_resolution),
            ((coord[0] + 1, coord[1] + 1), self.sqrtOf2 * self.map_resolution)
        ]

    def calculate_start_area_probabilities(self):
        self.start_area_probabilities = np.full(self.disconnected_possible_start_areas, 0, dtype=float)
        best_so_far = 0
        for idx, number_of_nodes in enumerate(self.nodes_in_block):
            self.start_area_probabilities[idx] = number_of_nodes / self.total_reachable_tiles_in_start_area
            if self.start_area_probabilities[idx] > best_so_far:
                best_so_far = self.start_area_probabilities[idx]
                self.most_likely_start_area_number = idx

    def update_area_probabilities_based_on_mean(self, new_area_probabilities):
        best_so_far = 0
        for idx, probability in enumerate(new_area_probabilities):
            diff = probability - self.start_area_probabilities[idx]
            self.start_area_probabilities[idx] = self.start_area_probabilities[
                                                     idx] + diff / self.probability_update_iteration
            if self.start_area_probabilities[idx] > best_so_far:
                best_so_far = self.start_area_probabilities[idx]
                self.most_likely_start_area_number = idx

        self.probability_update_iteration += 1

    def update_area_probabilities_multiply_normalized(self, new_area_probabilities):
        new_total_probabilities = 0
        best_so_far = 0
        for idx, (probability, start_prob) in enumerate(zip(new_area_probabilities, self.start_area_probabilities)):
            self.start_area_probabilities[idx] = probability * start_prob
            new_total_probabilities += self.start_area_probabilities[idx]

        for idx, probability in enumerate(self.start_area_probabilities):
            if probability > 0:
                self.start_area_probabilities[idx] = probability / new_total_probabilities
                if self.start_area_probabilities[idx] > best_so_far:
                    best_so_far = self.start_area_probabilities[idx]
                    self.most_likely_start_area_number = idx
        self.probability_update_iteration += 1

    def get_next_guard_position_when_guarding_area(self, start_area_index):
        adversary_coordinate = self.find_closest_valid_point(self.position_to_tuple(
            self.get_2d_position_from_odom(self.enemy_approximate_positions[-1])))
        # start area distances is a list of numpy arrays
        adversary_distance = self.start_area_distances[start_area_index][adversary_coordinate]
        return self.find_halfway_distance_position_from_coord_to_start_area(start_area_index, adversary_distance,
                                                                            adversary_coordinate)

    def find_halfway_distance_position_from_coord_to_start_area(self, start_area_index, distance, coord):
        short_distance = math.inf
        while self.start_area_distances[start_area_index][coord] > distance / 2:
            for (neighbour, _) in self.get_adjacent_fields(coord):
                if self.start_area_distances[start_area_index][neighbour] < short_distance:
                    short_distance = self.start_area_distances[start_area_index][neighbour]
                    coord = neighbour
        return coord

    def is_point_free(self, coord):
        is_free = self.is_coord_considered_free(coord)
        for n, _ in self.get_adjacent_fields(coord):
            is_free = is_free and self.is_coord_considered_free(n)

    def find_closest_valid_point(self, coord):
        open_nodes = [(coord[0], coord[1])]
        while open_nodes:
            top = open_nodes.pop(0)

            total_prob_to_reach_point = 0
            for i in range(0, self.disconnected_possible_start_areas):
                if self.start_area_distances[i][top] < math.inf and self.is_point_free(top):
                    total_prob_to_reach_point += self.start_area_probabilities[i]
                    if total_prob_to_reach_point >= 0.25:
                        return top

            for neighbour in self.get_simple_adjacent_fields(top):
                if self.is_tile_in_bounds(neighbour):
                    open_nodes.append(neighbour)

        return coord

    def probability_of_point_from_start_block_after_seconds(self, adversary_odometry, max_distance_traveled_so_far):
        start_x, start_y = self.get_2d_position_from_odom(adversary_odometry)
        start_index = self.position_to_index(start_x, start_y)
        total_points_reachable = 0
        probability_points_for_block = np.full(self.disconnected_possible_start_areas, 0, dtype=float)
        # check all tiles around the heard position where the adversary could possibly be
        for x in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1, 1):
            for y in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1, 1):
                coord = (start_index[0] + x, start_index[1] + y)
                if ((x * x + y * y) < self.sqr_max_noise_tile_error
                        and self.is_tile_in_bounds(coord)
                        and self.is_coord_considered_free(coord)):
                    blocks_that_reach_tile = []

                    # check how many blocks can reach that tile
                    for block_id in range(0, self.disconnected_possible_start_areas):
                        if (self.is_start_area_active(block_id) and
                                self.start_area_distances[block_id][coord] <= max_distance_traveled_so_far):
                            blocks_that_reach_tile.append(block_id)

                    number_blocks_that_reach_tile = len(blocks_that_reach_tile)
                    total_points_reachable += number_blocks_that_reach_tile

                    # update probabilty points for block based on number of blocks that can reach tile
                    for block_id in blocks_that_reach_tile:
                        if (self.is_start_area_active(block_id) and
                                self.start_area_distances[block_id][coord] <= max_distance_traveled_so_far):
                            probability_points_for_block[
                                block_id] += self.disconnected_possible_start_areas / number_blocks_that_reach_tile

        total_probability_points = total_points_reachable * self.disconnected_possible_start_areas
        if total_probability_points > 0:
            for idx, probability in enumerate(probability_points_for_block):
                probability_points_for_block[idx] = probability / total_probability_points
            self.update_area_probabilities_multiply_normalized(probability_points_for_block)

    def build_start_map(self, start_x, start_y):
        start_map = np.full((self.map_width, self.map_height), -1, dtype=int)
        start_index = self.position_to_index(start_x, start_y)
        for x in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1):
            for y in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1):
                coord = (start_index[0] + x, start_index[1] + y)
                if ((x * x + y * y) <= self.sqr_max_noise_tile_error
                        and self.is_tile_in_bounds(coord)):
                    start_map[coord] = self.get_grid_value_at_coord(coord)

        return start_map

    def search_separated_possible_start_areas(self, adversary_odometry):
        (start_x, start_y) = self.get_2d_position_from_odom(adversary_odometry)
        start_map = self.build_start_map(start_x, start_y)
        start_index = self.position_to_index(start_x, start_y)
        self.node_block_id = np.full((self.map_width, self.map_height), -1, dtype=int)
        block_count = 0
        for x in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1):
            for y in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1):
                coord = start_index[0] + x, start_index[1] + y
                if (self.node_block_id[coord] == -1
                        and self.is_coord_considered_free_in_map(coord, start_map)
                        and self.is_tile_in_bounds(coord)):
                    if self.bfs(coord[0], coord[1], block_count, start_map):
                        block_count += 1

        self.disconnected_possible_start_areas = block_count

    # starts a bfs from a start point and marks all valid fields with the given block id.
    # global_offset is the offset to add to the current coordinate to get the global coordinate
    # returns true if at least one field was marked
    def bfs(self, start_x, start_y, block_id, graph):
        open_nodes = [(start_x, start_y)]
        marked_nodes_count = 0
        while open_nodes:
            top = open_nodes.pop(0)
            marked_nodes_count += 1
            self.node_block_id[top] = block_id
            self.global_open_nodes.append(top)
            for neighbour in self.get_simple_adjacent_fields(top):
                if (self.node_block_id[neighbour] == -1
                        and self.is_coord_considered_free_in_map(neighbour, graph)
                        and self.is_tile_in_bounds(neighbour)):
                    open_nodes.append(neighbour)
                    # prevent the position being added to the open nodes more then once
                    self.node_block_id[neighbour] = -2
        if marked_nodes_count > 0:
            self.nodes_in_block.append(marked_nodes_count)
            self.total_reachable_tiles_in_start_area += marked_nodes_count
        return marked_nodes_count > 0

    def build_start_area_distances(self):
        open_nodes = []
        for i in range(0, self.disconnected_possible_start_areas):
            distances_to_block = np.full((self.map_width, self.map_height), math.inf, dtype=float)
            for n in self.global_open_nodes:
                if self.node_block_id[n] == i:
                    distances_to_block[n] = 0
                    open_nodes.append(n)
            while open_nodes:
                top = open_nodes.pop(0)
                for (p, dis) in self.get_adjacent_fields(top):
                    if self.is_tile_in_bounds(p) and self.is_coord_considered_free(p):
                        if distances_to_block[p] == math.inf:
                            open_nodes.append(p)
                        distance = distances_to_block[top] + dis
                        if distance < distances_to_block[p]:
                            distances_to_block[p] = distance
            self.start_area_distances.append(distances_to_block)


if __name__ == '__main__':
    rospy.init_node('start_area')
    StartAreasModel()
    rospy.spin()
