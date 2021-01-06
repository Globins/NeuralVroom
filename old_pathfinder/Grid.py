from .vehicle import *
from .GVD import GVD
from .utils import *

#initialize the grid with a specified size for a grid, does not need to be uniform
#set up obstacles using the addobstacle and removeobstacle
class Grid:
    """Grid interface that the main program will be using to update and control the processing map"""
    def __init__(self, width:int = 0, height:int = 0):
        self.width = width
        self.height = height
        self.GVD = GVD(width, height)
        self.occupiedCells = [[0 for i in range(width)] for j in range(height)]
        self.obstacleCells = []
        
    def AddObstacle(self, point:tuple = (-1, -1)):
        """Adds an obstacle to the grid"""
        if(IsPointInGrid(point, (self.width, self.height)) and not point in self.obstacleCells):
            self.occupiedCells[point[0]][point[1]] = 1
            self.obstacleCells.append(point)
            self.GVD.SetCell(point)
            self.GVD.Update()
            
    def RemoveObstacle(self, point:tuple):
        """removes an obstacle from the grid"""
        if(IsPointInGrid(point, (self.width, self.height)) and point in self.obstacleCells):
            self.occupiedCells[point[0]][point[1]] = 0
            self.obstacleCells.remove(point)
            self.GVD.UnSetCell(point)
            self.GVD.Update()
            
    def RefreshMap(self):
        """Updates the map manually"""
        self.GVD.Update()
    
    def returnDistanceMap(self):
        """returns a double array of int distances from obstacles"""
        distMap = self.GVD.distMap
        dist = []
        for x in range(self.width):
            row = []
            for y in range(self.height):
                row.append(distMap[x][y].dist)
            dist.append(row)
        return dist
    
    def returnVoroDistanceMap(self):
        """returns a double array of int distances from the voronoi lines"""
        vDistMap = self.GVD.voroDistMap
        vDist = []
        for x in range(self.width):
            row = []
            for y in range(self.height):
                row.append(vDistMap[x][y].dist)
            vDist.append(row)
        return vDist
    def returnVoroMap(self):
        """returns a double array of bools of whether it is in a voronoi border or not"""
        voroMap = self.GVD.voro
        voro = []
        for x in range(self.width):
            row = []
            for y in range(self.height):
                row.append(voroMap[x][y])
            voro.append(row)
        return voro
    
    def returnCostMap(self):
        pass
    
    def IsSafe(self, state:VehicleState, safetyFactor):
        x = int(np.floor(state.position[0]))
        y = int(np.floor(state.position[1]))
        dist = self.returnDistanceMap()
        if(dist[x][y] < safetyFactor):
            return False
        return True
    