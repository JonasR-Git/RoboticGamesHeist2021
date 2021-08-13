import numpy as np
import math


class StartAreasModel:

    def __init__(self, start_position):
        # occupancy grid stuff and config of occupancy grid
        self.occupancy_grid = np.array((0, 0))
        self.occupancy_grid_tile_size = 0.05
        self.occupancy_grid_size = 200
        self.max_error_distance = 1
        self.max_noise_tile_error = math.ceil(self.max_error_distance / self.occupancy_grid_tile_size)
        self.sqr_max_noise_tile_error = self.max_noise_tile_error ** 2
        self.start_map_size = self.max_noise_tile_error * 2 + 1
        self.is_reachable_threshold = 0.3
        self.sqrtOf2 = math.sqrt(2)

        # if a start area has the likelihood of under a half percent it is rejected
        self.threshold_to_reject_start_area = 0.005

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

        self.search_separated_possible_start_areas(start_position[0], start_position[1])
        self.calculate_start_area_probabilities()
        self.global_open_nodes = []

    def is_tile_in_bounds(self, coord):
        return 0 <= coord[0] < self.occupancy_grid_size and 0 <= coord[1] < self.occupancy_grid_size

    def is_start_area_active(self, block_id):
        return self.start_area_probabilities[block_id] < self.threshold_to_reject_start_area

    def position_to_index(self, x, y):
        return int(round(x / self.occupancy_grid_tile_size)), int(round(y / self.occupancy_grid_tile_size))

    def is_coord_considered_free(self, coord):
        return coord >= 0 and self.occupancy_grid[coord] < self.is_reachable_threshold

    def is_coord_considered_free_in_map(self, coord, map_view):
        return coord >= 0 and map_view[coord] < self.is_reachable_threshold

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
            ((coord[0] - 1, coord[1] - 1), self.sqrtOf2),
            ((coord[0] - 1, coord[1]), 1),
            ((coord[0] - 1, coord[1] + 1), self.sqrtOf2),
            ((coord[0], coord[1] - 1), 1),
            ((coord[0], coord[1] + 1), 1),
            ((coord[0] + 1, coord[1] - 1), self.sqrtOf2),
            ((coord[0] + 1, coord[1]), 1),
            ((coord[0] + 1, coord[1] + 1), self.sqrtOf2)
        ]

    def calculate_start_area_probabilities(self):
        self.start_area_probabilities = np.full(self.disconnected_possible_start_areas, 0, dtype=float)
        i = 0
        for n in self.nodes_in_block:
            self.start_area_probabilities[i] = len(n) / self.total_reachable_tiles_in_start_area
            i += 1

    def update_area_probabilities_based_on_mean(self, new_area_probabilities):
        i = 0
        for probability in new_area_probabilities:
            diff = probability - self.start_area_probabilities[i]
            self.start_area_probabilities[i] = self.start_area_probabilities[
                                                   i] + diff / self.probability_update_iteration
            i += 1

        self.probability_update_iteration += 1

    def update_area_probabilities_multiply_normalized(self, new_area_probabilities):
        new_total_probabilities = 0
        for idx, (probability, start_prob) in enumerate(zip(new_area_probabilities, self.start_area_probabilities)):
            self.start_area_probabilities[idx] = probability * start_prob
            new_total_probabilities += start_prob

        for idx, probability in enumerate(self.start_area_probabilities):
            self.start_area_probabilities[idx] = probability / new_total_probabilities

    def probability_of_point_from_start_block_after_seconds(self, position, block_index, max_distance_traveled_so_far):
        total_points_reachable = 0
        probability_points_for_block = np.full(self.disconnected_possible_start_areas, 0, dtype=int)
        start_index = self.position_to_index(position[0], position[1])
        # check all tiles around the heard position where the adversary could possibly be
        for x in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1, 1):
            for y in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1, 1):
                coord = (start_index[0] + x, start_index[1] + y)
                if ((x * x + y * y) / self.occupancy_grid_tile_size < self.sqr_max_noise_tile_error
                        and self.is_tile_in_bounds(coord)
                        and self.is_coord_considered_free(coord)):
                    blocks_that_reach_tile = []

                    for block_id in range(0, self.disconnected_possible_start_areas):
                        if (self.is_start_area_active(block_id) and
                                self.start_area_distances[block_id][coord] <= max_distance_traveled_so_far):
                            blocks_that_reach_tile.append(block_id)

                    number_blocks_that_reach_tile = len(blocks_that_reach_tile)
                    total_points_reachable += number_blocks_that_reach_tile

                    for block_id in blocks_that_reach_tile:
                        if (self.is_start_area_active(block_id) and
                                self.start_area_distances[block_id][coord] <= max_distance_traveled_so_far):
                            probability_points_for_block[
                                block_id] += self.disconnected_possible_start_areas / number_blocks_that_reach_tile

        total_probability_points = total_points_reachable * self.disconnected_possible_start_areas
        for idx, probability in enumerate(probability_points_for_block):
            probability_points_for_block[idx] = probability / total_probability_points

        self.update_area_probabilities_based_on_mean(probability_points_for_block)

    def build_start_map(self, start_x, start_y):
        start_map = np.full((self.occupancy_grid_size, self.occupancy_grid_size), -1, dtype=int)
        start_index = self.position_to_index(start_x, start_y)
        start_map_index = (self.max_noise_tile_error, self.max_noise_tile_error)
        for x in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1, 1):
            for y in range(-self.max_noise_tile_error, self.max_noise_tile_error + 1, 1):
                coord = (start_index[0] + x, start_index[1] + y)
                if ((x * x + y * y) / self.occupancy_grid_tile_size < self.sqr_max_noise_tile_error
                        and self.is_tile_in_bounds(coord)):
                    start_map[coord] = self.occupancy_grid[coord]

        return start_map

    def search_separated_possible_start_areas(self, start_x, start_y):
        start_map = self.build_start_map(start_x, start_y)
        start_index = self.position_to_index(start_x, start_y)
        self.node_block_id = np.full((self.occupancy_grid_size, self.occupancy_grid_size), -1, dtype=int)
        block_count = 0
        for x in range(0, self.start_map_size, 1):
            for y in range(0, self.start_map_size, 1):
                coord = start_index[0] + x, start_index[1] + y
                if (self.node_block_id[coord] == -1
                        and self.is_coord_considered_free_in_map(coord, start_map)
                        and self.is_tile_in_bounds(coord)):
                    if self.bfs(start_x, start_y, block_count, start_map):
                        block_count += 1

        self.disconnected_possible_start_areas = block_count

    # starts a bfs from a start point and marks all valid fields with the given block id.
    # global_offset is the offset to add to the current coordinate to get the global coordinate
    # returns true if at least one field was marked
    def bfs(self, start_x, start_y, block_id, graph):
        open_nodes = [(start_x, start_y)]
        marked_nodes_count = 0
        while open_nodes:
            top = open_nodes.pop()
            marked_nodes_count += 1
            self.node_block_id[top] = block_id
            self.global_open_nodes.append(top)
            for neighbour in self.get_adjacent_fields(top):
                if (self.node_block_id[neighbour] == -1
                        and self.is_coord_considered_free_in_map(neighbour, graph)
                        and self.is_tile_in_bounds(neighbour)):
                    open_nodes.append(neighbour)
                    # prevent the position being added to the open nodes more then once
                    self.node_block_id[neighbour] = -2
        if marked_nodes_count > 0:
            self.nodes_in_block.append(marked_nodes_count)
            self.total_reachable_tiles_in_start_area += 1
            self.total_reachable_tiles_in_start_area += marked_nodes_count
        return marked_nodes_count > 0

    def build_start_area_distances(self):
        open_nodes = []
        for i in range(0, self.disconnected_possible_start_areas):
            distances_to_block = np.full((self.occupancy_grid_size, self.occupancy_grid_size), -1, dtype=float)
            for n in self.global_open_nodes:
                if self.node_block_id[n] == i:
                    distances_to_block[n] = 0
                    open_nodes.append(n)

            while open_nodes:
                top = open_nodes.pop()
                for (p, dis) in self.get_adjacent_fields(top):
                    if self.is_tile_in_bounds(p) and self.is_coord_considered_free(p):
                        if self.start_area_distances[i][p] == -1:
                            open_nodes.append(p)

                        distance = self.start_area_distances[i][top] + dis
                        if distance < self.start_area_distances[i][p]:
                            self.start_area_distances[i][p] = distance

            self.start_area_distances.append(distances_to_block)
