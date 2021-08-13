import heapq
import numpy as np
import math

class start_areas_modell:

    def __init__(self, start_position):
        #occupancy grid stuff and config of occupancy grid
        self.occupancyGrid = []
        self.occupancyGridTileSize = 0.05
        self.occupancyGridSize = 200
        self.max_error_distance = 1
        self.maxNoiseTileError = math.ceil(self.max_error_distance / self.occupancyGridTileSize)
        self.sqr_max_noise_tile_error = self.maxNoiseTileError ** 2
        self.start_map_size = self.maxNoiseTileError * 2 + 1
        self.is_reachable_threshhold = 0.3
        self.sqrtOf2 = math.sqrt(2)
        
        #if a startarea has the likelyhood of under a half percent it is rejected
        self.threshold_to_reject_start_area = 0.005

        self.total_reachable_tiles_in_start_area = 0
        self.disconnected_possible_start_areas = 0
        self.node_block_id = []
        #value at index i is number of tiles in start area i
        self.nodes_in_block = []
        #list of a 2d array
        #each entry i stores a grid containing the distance from every node to the starting area i
        self.start_area_distances = []
        self.start_area_probabilities = []
        self.most_likely_start_area_number = 0

        self.probability_update_iteration = 1

        self.search_seperated_possible_start_areas(start_position[0], start_position[1])
        self.calculate_start_area_probabilties()
       

    def is_tile_in_bounds(self, coord):
        return coord[0] >= 0 and coord[1] >= 0 and coord[0] < self.occupancyGridSize and coord[1] < self.occupancyGridSize

    def is_start_area_active(self, id):
        return self.start_area_probabilities[block_id] < self.threshold_to_reject_start_area

    def position_to_index(self, x, y):
        return (x / self.occupancyGridTileSize, y / self.occupancyGridTileSize)

    def is_coord_considered_free(self, coord):
        return coord >= 0 and self.occupancyGrid[coord] < self.is_reachable_threshhold

    def is_coord_considered_free_in_map(self, coord, map):
        return coord >= 0 and map[coord] < self.is_reachable_threshhold

    def sqr_magnitude(self, coord):
        return coord[0] * coord[0] + coord[1] * coord[1]

    def get_simple_adjacent_fields(self, coord):
       return [
            (coord[0] - 1, coord[1]),
            (coord[0], coord[1] - 1),
            (coord[0], coord[1] + 1),
            (coord[0] + 1, coord[1]),
            ]

    def get_adjacent_fields(self, coord):
        return [
            ((coord[0] - 1, coord[1] - 1), self.sqrtOf2),
            ((coord[0] - 1, coord[1] ), 1),
            ((coord[0] - 1, coord[1] + 1), self.sqrtOf2),
            ((coord[0], coord[1] - 1 ), 1),
            ((coord[0], coord[1] + 1 ), 1),
            ((coord[0] + 1, coord[1] - 1), self.sqrtOf2),
            ((coord[0] + 1, coord[1] ), 1),
            ((coord[0] + 1, coord[1] + 1), self.sqrtOf2)
            ]


    def calculate_start_area_probabilties(self):
        self.start_area_probabilities = np.full(self.disconnected_possible_start_areas, 0, dtype=float)
        for n in self.nodes_in_block:
            self.start_area_probabilities[i] = len(n) / self.total_reachable_tiles_in_start_area
      
    def update_area_probabilities_based_on_mean(self, new_area_probabilities):
        i = 0
        for probability in new_area_probabilities:
            diff = probability - self.start_area_probabilities[i]
            self.start_area_probabilities[i] = self.start_area_probabilities[i] + diff / self.probability_update_iteration
            i += 1

        self.probability_update_iteration += 1

    def update_area_probabilities_multiply_normalized(self, new_area_probabilities):
        new_total_probabilities = 0
        i = 0
        for probability in new_area_probabilities:
            self.start_area_probabilities[i] = probability * self.start_area_probabilities[i]
            new_total_probabilities += self.start_area_probabilities[i]
            i += 1

        i = 0
        for probability in self.start_area_probabilities:
            self.start_area_probabilities[i] = self.start_area_probabilities[i] / new_total_probabilities
            i += 1

    def probability_of_point_from_start_block_after_seconds(self, position, block_index, max_distance_traveled_so_far):
        total_points_reachable = 0
        probability_points_for_block = np.full(self.disconnected_possible_start_areas, 0, dtype = int)
        start_index = self.position_to_index(startX, startY)
        #check all tiles around the heared position where the adversary could possibly be
        for x in range(-self.maxNoiseTileError, self.maxNoiseTileError + 1, 1):
            for y in range(-self.maxNoiseTileError, self.maxNoiseTileError + 1, 1):
                coord = (start_index[0] + x, start_index[1] + y)
                if ((x * x + y * y) / GRID_BLOCK_SIZE < self.sqr_max_noise_tile_error
                        and self.is_tile_in_bounds(coord)
                        and self.is_coord_considered_free(coord)):
                    blocks_that_reach_tile = []

                    for block_id in range(0, self.disconnected_possible_start_areas):
                        if(self.is_start_area_active(block_id) and 
                                self.start_area_distances[i][coord] <= max_distance_traveled_so_far):
                                blocks_could_reach_tile.append(block_id)

                    number_blocks_that_reach_tile = len(blocks_that_reach_tile)
                    total_points_reachable += number_blocks_that_reach_tile

                    for block_id in blocks_could_reach_tile:
                        if(self.is_start_area_active(block_id) and 
                                self.start_area_distances[i][coord] <= max_distance_traveled_so_far):
                            blocks_could_reach_tile.append(block_id)
                            probability_points_for_block[block_id] += self.disconnected_possible_start_areas / number_blocks_that_reach_tile
        
        total_probability_points = total_points_reachable * self.disconnected_possible_start_areas
        i = 0

        for probability in probability_points_for_block:
            probability_points_for_block[i] = probability_points_for_block[i] / total_probability_points
            i += 1

        self.update_area_probabilities(probability_points_for_block)
        


    def build_start_map(self, startX, startY):
        startMap = np.full((self.occupancyGridSize, self.occupancyGridSize), -1, dtype=int)
        start_index = self.position_to_index(startX, startY)
        start_map_index = (self.maxNoiseTileError, self.maxNoiseTileError)
        for x in range(-self.maxNoiseTileError, self.maxNoiseTileError + 1, 1):
            for y in range(-self.maxNoiseTileError, self.maxNoiseTileError + 1, 1):
                coord = (start_index[0] + x, start_index[1] + y)
                if ((x * x + y * y) / self.occupancyGridTileSize < self.sqr_max_noise_tile_error
                        and self.is_tile_in_bounds(coord)):
                    startMap[coord]=self.occupancyGrid[coord]

        return startMap

    def search_seperated_possible_start_areas(self, startX, startY):
        startMap = self.build_start_map(startX, startY)
        start_index = self.position_to_index(startX, startY)
        self.node_block_id = np.full((self.occupancyGridSize, self.occupancyGridSize), -1, dtype = int)
        blockCount = 0
        for x in range(0, self.start_map_size, 1):
            for y in range(0, self.start_map_size, 1):
                coord = start_index[0] + x, start_index[1] + y
                if(self.node_block_id[coord] == -1 and self.is_coord_considered_free_in_map(coord, map) and self.is_tile_in_bounds(coord)):
                    if(self.bfs(startX, startY, blockCount, startMap)):
                        blockCount += 1

        self.disconnected_possible_start_areas = blockCount


    #starts a bfs from a start point and marks all valid fields with the given block id.
    #global_offset is the offset to add to the current coordinate to get the global coordinate
    #returns true if at least one field was marked
    def bfs(self, startX, startY, blockId, map):
        open_nodes = [(startX, startY)]
        reached_nodes = []
        marked_nodes_count = 0
        while(not open_nodes.empty()):
            top = open_nodes.pop()
            marked_nodes_count += 1
            reached_nodes.append(global_top)
            self.node_block_id[globalTop] = nodeBlockId
            self.global_open_nodes.push(globalTop)
            for neighbor in self.get_adjacent_fields(top):
                if(self.node_block_id[neighbor] == -1 and self.is_coord_considered_free_in_map(neighbor, map) and self.is_tile_in_bounds(neighbor)):
                    open_nodes.push(neighbour)
                    #prevent the position being added to the open nodes more then once
                    self.node_block_id[neighbor] = -2

        if(marked_nodes_count > 0):
            self.nodes_in_block.append(marked_nodes_count)
            total_reachable_tiles_in_start_area += 1
            self.total_reachable_tiles_in_start_area += marked_nodes_count
            
        return marked_nodes_count > 0
        
    

    def build_start_area_distances(self):
        open_nodes = []
        for i in ranges(0,self.disconnected_possible_start_areas):
            distances_to_block = np.full((self.occupancyGridSize, self.occupancyGridSize), -1, dtype = float)
            for n in self.global_open_nodes:
                if(self.node_block_id[n] == i):
                    distances_to_block[n] = 0
                    open_nodes.push(n)

            while(not open_nodes.empty()):
                top = open_nodes.pop()
                for (p, dis) in self.get_adjacent_fields(top):
                    if(self.is_tile_in_bounds(p) and is_coord_considered_free(p)):
                        if(self.distances_to_block[p] == -1):
                             open_nodes.push(p)

                        distance = self.distances_to_block[top] + dis
                        if(distance < self.distances_to_block[p]):
                            self.distances_to_block[p] = distance

            self.start_area_distances.append(distances_to_block)
