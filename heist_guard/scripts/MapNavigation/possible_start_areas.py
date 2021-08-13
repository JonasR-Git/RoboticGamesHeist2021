from casadi import *
import heapq
import numpy as np
import math


class points:

    def __init__(self):
        self.pointSet = []
        self.seperatedPossibleStartAreas = 0
        self.occupancyGrid = []
        self.node_block_id_lookup = []
        self.start_area_distances = []
        self.start_block_nodes = []
        self.occupancyGridTileSize = 0.05
        self.occupancyGridSize = 200
        self.maxNoiseTileError = 1 / self.occupancyGridTileSize
        self.start_map_size = maxNoiseTileError * 2 + 1
        self.sqrtOf2 = math.sqrt(2)
        self.startX = 0
        self.startY = 0
        self.is_driveable_threshhold = 1

    def is_tile_in_bounds(self, coord):
        return coord[0] >= 0 and coord[1] >= 0 and coord[0] < self.mapGridSize and coord[1] < self.mapGridSize

    def position_to_index(self, x, y):
        return (x / self.occupancyGridTileSize, y / self.occupancyGridTileSize)

    def get_start_map_part(self, startX, startY):
        self.startX = startX
        self.startY = startY
        startMap = np.full((self.start_map_size, self.start_map_size), -1, dtype=int)
        start_index = self.position_to_index(startX, startY)
        start_map_index = (maxNoiseTileError, maxNoiseTileError)
        for x in range(-maxNoiseTileError, maxNoiseTileError + 1, 1):
            for y in range(-maxNoiseTileError, maxNoiseTileError + 1, 1):
                if ((x * x + y * y) / GRID_BLOCK_SIZE < maxNoiseTileError
                        and self.is_tile_in_bounds((start_index + x, start_index + y))):
                    startMap[start_map_index[0] + x, start_map_index[1] + y]=occupancyGrid[start_index[0] + x, start_index[1] + y]

        return startMap

    def count_seperated_possible_start_areas(self, startX, startY):
        startMap = self.get_start_map_part(startX, startY)
        start_index = self.position_to_index(startX, startY)
        self.node_block_id = np.full((self.occupancyGridSize, self.occupancyGridSize), -1, dtype = int)
        blockCount = 0
        for x in range(0, self.start_map_size, 1):
            for y in range(0, self.start_map_size, 1):
                if(self.node_block_id[start_index[0] + x, start_index[1] + y] == -1 and startMap[x,y] > 0):
                    if(self.bfs(startX, startY, blockCount, startMap, nodeBlockIds)):
                        blockCount += 1

        self.seperatedPossibleStartAreas = blockCount


    def build_start_area_distances(self):
        open_nodes = []
        for i in ranges(0,self.seperatedPossibleStartAreas):
            distances_to_block = np.full((self.occupancyGridSize, self.occupancyGridSize), -1, dtype = float)
            for n in self.global_open_nodes:
                if(self.node_block_id_lookup[n] == i):
                    distances_to_block[n] = 0
                    open_nodes.push(n)

            while(not open_nodes.empty()):
                top = open_nodes.pop()
                for (p, dis) in self.get_adjacent_fields(top):
                    if(self.is_tile_in_bounds(p) and occupancyGrid[p] > self.is_driveable_threshhold):
                        if(self.distances_to_block[p] == -1):
                             open_nodes.push(p)

                        distance = self.distances_to_block[top] + dis
                        if(distance < self.distances_to_block[p]):
                            self.distances_to_block[p] = distance

            self.start_area_distances.append(distances_to_block)



    def get_simple_adjacent_fields(self, coord):
        return [
            (coord[0] - 1, coord[1] ),
            (coord[0], coord[1] - 1 ),
            (coord[0], coord[1] + 1 ),
            (coord[0] + 1, coord[1] ),
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

    #starts a bfs from a start point and marks all valid fields with the given block id.
    #global_offset is the offset to add to the current coordinate to get the global coordinate
    #returns true if at least one field was marked
    def bfs(self, startX, startY, blockId, map, global_offset):
        open_nodes = [(startX, startY)]
        marked_at_least_one = False
        while(not open_nodes.empty()):
            top = open_nodes.pop()
            globalTop = (top[0] + global_offset[0], top[1] + global_offset[1])
            #open_nodes.pop()
            #check if tile is in bounds,
            if(self.is_tile_in_bounds(globalTop) and map[top] > self.is_driveable_threshhold and self.node_block_id[globalTop] == -1)):
                marked_at_least_one = True
                self.node_block_id[globalTop] = nodeBlockId
                self.global_open_nodes.push(globalTop)
                for(neighbor in self.get_adjacent_fields(top)):
                    open_nodes.push(neighbour)

        return marked_at_least_one
