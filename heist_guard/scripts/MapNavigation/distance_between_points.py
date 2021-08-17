import numpy as np
import math


class Pathfinder:

    def __init__(self, mapGridSize):
        self.map = None
        self.mapGridSize = mapGridSize
        self.sqrtOf2 = math.sqrt(2)
        self.distanceToReachTiles = None
        self.tails = None
        self.targetCoord = None
        self.distanceToGrid = None
        self.parentOfGridX = None
        self.parentOfGridY = None
        self.found_obstacles_to_target = None

    def is_point_between(self, x1, y1, x2 , y2):


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
        return (self.is_tile_in_bounds(fieldCoord)
        #check if tile is marked as reachable
        and self.map[fieldCoord])

    def is_new_field_valid(self, fieldCoord):
        return (self.is_field_valid
        #check that field was not visited already
        and self.parentOfGridX[fieldCoord] == None)


    def get_min_distance_to_target(self, fromCoord):
        xDiff = fromCoord[0] - targetCoord[0]
        yDiff = fromCoord[1] - targetCoord[1]
        biggerDiff = math.max(xDiff, yDiff)
        smallerDiff = math.min(xDiff, yDiff)
        return self.sqrtOf2 * smallerDiff + 1 * (biggerDiff - smallerDiff)

    def add_field_to_tails(self, fieldCoord, parentCoord, previousDistance):
        minDistance = get_min_distance_to_target(fieldCoord)
        total_distance = previousDistance + minDistance
        self.parentOfGridX[fieldCoord] = parentCoord[0]
        self.parentOfGridY[fieldCoord] = parentCoord[1]
        self.distanceToGrid[fieldCoord] = previousDistance
        heappush(self.tails, (total_distance, fieldCoord))

    def advance_closest(self):
        closestCoord = heappop(self.tails)[1]
        previousDistance = self.distanceToGrid[closestCoord]
        for neighbourInfo in get_adjacent_fields(closestCoord):
            if(is_new_field_valid(neighbourInfo[0])):
                field_evaluation = neighbourInfo[1] + previousDistance
                add_field_to_tails(neighbourInfo[0], closestCoord, field_evaluation)

    def has_reached_target(self):
        return self.parentOfGridX[self.targetCoord] >= 0

    def get_path(self):
        path = []
        lastParent = self.targetCoord
        nextParent = (self.parentOfGridX[lastParent], self.parentOfGridY[lastParent])
        path.append(lastParent)
        while(lastParent != nextParent):
            lastParent = nextParent
            nextParent = (self.parentOfGridX[lastParent], self.parentOfGridY[lastParent])
            path.append(lastParent)

    #returns a tuple, where the first entry is a bool, if a path was found
    #the second entry is a stack with all the map coordinates in the shortest path
    def get_path_to(self, startCoord, toCoord, map):
        self.map = map
        self.targetCoord = toCoord
        if not is_field_valid(startCoord) or not is_field_valid(toCoord):
            return (false, [])

        result = (None, None)
        self.found_obstacles_to_target = []
        self.distanceToGrid = np.zeros((self.mapGridSize, self.mapGridSize), dtype=float)
        self.parentOfGridX = np.full((self.mapGridSize, self.mapGridSize), -1, dtype=int)
        self.parentOfGridY = np.full((self.mapGridSize, self.mapGridSize), -1, dtype=int)
        self.tails = []

        add_field_to_tails(startCoord,startCoord)

        self.closest = fromCoord
        while(not has_reached_target() and len(self.tails > 0)):
            advance_closest()

        if has_reached_target():
            result = (true, get_path())
        else :
            result = (false,[])

        self.distanceToGrid = None
        self.parentOfGridX = None
        self.parentOfGridY = None

        return result
