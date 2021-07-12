from casadi import *
import numpy as np
import math



class map_builder:

    __slots__ = [
        "mapSize", 
        "roboterSize", 
        "halfedRoboterSquaredSize", 
        "gridBlockSize", 
        "mapTilesPerAxis", 
        "map", 
        "xPos", 
        "yPos", 
        "orientation"]

    def __init__(self):
        self.mapSize = 1000
        self.roboterSize = 15
        self.halfedRoboterSquaredSize = (roboterSize/ 2) ** 2
        self.gridBlockSize = 5
        self.mapTilesPerAxis = mapSize / gridBlockSize
        self.map = np.ones((mapTilesPerAxis, mapTilesPerAxis), dtype=bool)
        self.position = (0,0)
        self.orientation = 0

        #forward vector of roboter with rotation = 0
        self.defaultForwardX = 1
        self.defaultForwardY = 0

        rospy.Subscriber("scan", LaserScan, self.scan_callback)

    def local_to_global_orientation(self, localOrientation):
        return self.orientation + w
    
    def is_tile_in_bounds(self, x, y):
        return x >= 0 and y >= 0 and x < self.mapTilesPerAxis and y < self.mapTilesPerAxis

    def get_rot(self, orientation):
        return orientation # * 2 * math.pi

    def get_direction_from_orientation(self, orientation):
        result = np.array(1.0,1.0)
        radians = get_rot(orientation)
        #forward direction x value
        result[0] = self.defaultForwardX * math.cos(radians) - self.defaultForwardY * math.sin(radians) 
        #forward direction y value
        result[1] = self.defaultForwardX * math.sin(radians) + self.defaultForwardY * math.cos(radians) 
        return result

    def add_point_to_map(self, obstacleDistance, obstacleLocalOrientation):
        globalOrientation = local_to_global_orientation(obstacleLocalOrientation)
        obstacleDirection = get_direction_from_orientation(globalOrientation)
        obstacleX = int(round(self.xPos + obstacleDirection[0] * obstacleDistance))
        obstacleY = int(round(self.yPos + obstacleDirection[1] * obstacleDistance))

        #mark all spots as not reachable in a circle of half the roboters
        #square distance as radius as not reachable

        #maybe only do this when the field is marked as reachable
        #if(not map[obstacleX][obstacleY]):
        offsetSize = int(self.roboterSize / (self.gridBlockSize * 2))
        for x in range(-offsetSize, offsetSize + 1, 1):
            for y in range(-offsetSize, offsetSize + 1, 1):
                #check if distance from obstacle point is within squared roboter radius
                #maybe round here
                if((x * x + y * y) / self.gridBlockSize < self.halfedRoboterSquaredSize
                    and is_tile_in_bounds(x + obstacleX,y + obstacleY)):
                    map[x + obstacleX,y + obstacleY] = false

 
    def find_path_from_to(self, fromX, fromY, toX, toY):
        if(not is_tile_in_bounds(fromX, fromY) or not is_tile_in_bounds(toX,toY)):
            return
        

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        radians_increment = msg.angle_increment
        currentRadians = msg.angle_min

        for range in ranges:
            if(not math.isfinite(range)):
                add_point_to_map(range, currentRadians)
            currentRadians += radians_increment
                
            
            