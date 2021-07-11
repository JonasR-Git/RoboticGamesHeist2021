from casadi import *
import heapq
import numpy as np
import math


class PathPart:

    def __init__(self, parent = None, x = 0, y = 0):
        self.minDistance
        self.distanceToReachThis
        self.parent = parent
        self.position = (x, y)

    #only rebuild path if one tile on th path is not reachable anymore

    #return for each neighbour entry 3 values in an array
    #the first entry is neighbour x position, second is neighbour
    #y position and third is neighbour distance
    


    def min_absolute_distance(self):
        return self.minDistance + self.distanceToReachThis()

class Pathfinder:

    def __init__(self, mapGridSize):
        self.map
        self.mapGridSize = mapGridSize
        self.distanceToReachTiles = np.array()
        self.tails
        self.sqrtOf2 = math.sqrt(2)
        self.targetCoord
        self.distanceFromGridsToTarget
        self.distanceToGrid
        self.parentOfGrid
        
    #return a list of tuple
    #the first entry is another tuple representing the position of the field
    #the second entry is the distance to the field
    #Todo: disallow diagonal neighbours if either other diagonal field is not reachable
    #11
    #01 <- diagonal 1 fields are not connected!
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


    def is_tile_in_bounds(self, x, y):
        return x >= 0 and y >= 0 and x < self.mapGridSize and y < self.mapGridSize
 

    def is_field_valid(self, fieldCoord):
        return (is_tile_in_bounds(fieldCoord) 
        #check if tile is marked as reachable
        and self.map[fieldCoord])

    def is_new_field_valid(self, fieldCoord):
        return (is_field_valid
        #check that field was not visited already
        and self.distanceFromGridsToTarget[fieldCoord] == 0)


    def get_min_distance_to_target(self, fromCoord):
        xDiff = fromCoord[0] - targetCoord[0]
        yDiff = fromCoord[1] - targetCoord[1]
        biggerDiff = math.max(xDiff, yDiff)
        smallerDiff = math.min(xDiff, yDiff)
        return self.sqrtOf2 * smallerDiff + 1 * (biggerDiff - smallerDiff) 

    def add_field_to_tails(self, fieldCoord, parentCoord, previousDistance):
        minDistance = get_min_distance_to_target(fieldCoord)
        total_distance = previousDistance + minDistance
        self.parentOfGrid[fieldCoord] = parentCoord
        self.distanceToGrid[fieldCoord] = previousDistance
        self.distanceFromGridsToTarget[fieldCoord] = minDistance
        heappush(self.tails, (total_distance, fieldCoord))

    def advance_closest(self):
        closestCoord = heappop(self.tails)[1]
        previousDistance = self.distanceFromGridsToTarget[closestCoord]
        for neighbourInfo in get_adjacent_fields(closestCoord):
            if(is_new_field_valid(neighbourInfo[0])):
                field_evaluation = neighbourInfo[1] + previousDistance
                add_field_to_tails(neighbourInfo[0], closestCoord, field_evaluation)

    def has_reached_target(self):
        return self.parentOfGrid[self.targetCoord] != None

    def buid_path(self):
        path = []
        lastParent = self.targetCoord
        nextParent = self.parentOfGrid[lastParent]
        path.append(lastParent)
        while(lastParent != nextParent):
            lastParent = nextParent
            nextParent = self.parentOfGrid[lastParent]
            path.append(lastParent)

    def get_path_to(self, startCoord, toCoord, map):
        self.map = map
        self.targetCoord = toCoord
        if not is_field_valid(startCoord) or not is_field_valid(toCoord):
            return (false, [])
        
        self.distanceToGrid = np.zeros((self.mapGridSize, self.mapGridSize), dtype=float)
        self.distanceFromGridsToTarget = np.zeros((self.mapGridSize, self.mapGridSize), dtype=float)
        self.parentOfGrid = np.array((self.mapGridSize, self.mapGridSize), dtype=(int, int), like=None)
        self.tails = []

        add_field_to_tails(startCoord,startCoord)
        
        self.closest = fromCoord
        while(not has_reached_target and len(self.tails > 0)):


    

    def lower_bound_distance(fromX, fromY, toX, toY):
        return math.sqrt((fromX - toX) ** 2, (fromY - toY) ** 2)

    