from queue import PriorityQueue
import math
from .utils import *


#Generates and outptus a usable Generalized Voronoi Diagram Based on 
#http://www.first-mm.eu/files/lau10iros.pdf
# Used to calculate The best center path for the pathfinding algorithm to create smoother turns
#How it works:
#The map is initially a distance map with all infinite distances
#The map updates w/ set and unset obstacle this would be from the user,
#the map when updated, will calculate the distances
#when it detects a cell edge (when the distance stops updating), it sets a voro edge
#The voronoi map is then updated the same way the distance map is with the voro edges
class Cell:
    def __init__(self):
        self.dist = float('inf')
        #Obst/Voro
        self.nearest = (-1,-1)
        self.toRaise = False
        self.toProcess = False


class GVD:
    """all cells in distance map are infinite and cleared on init"""
    def __init__(self, width, height):
        self._unknown = (-1, -1) 
        self.width = width
        self.height = height
        # queue = (distance, (x,y))
        self.distOpen = PriorityQueue()
        self.distMap = [[Cell() for y in range(self.height )] for x in range(self.width)]  
        self.voroOpen = PriorityQueue()
        self.voroDistMap = [[Cell() for y in range(self.height )] for x in range(self.width)]
        #GVD FLags, will print true where the voro lines are
        self.voro = [[True for y in range(self.height )] for x in range(self.width)]
        
        #comp will be used in cost computing
        self.voroQ = PriorityQueue()
        self.comp = [[-1 for y in range(self.height )] for x in range(self.width)]
        #path cost
        self.pathCost = [[0 for y in range(self.height )] for x in range(self.width)]
        #Used to calculate cost difference in terms of distance from obstacles
        self.AlphaDefault = 20
        self.DMaxDefault = 30
        
    
    def Update(self):
        """Call to this will update the grid"""
        self._updateMap()
        self._updateMap(isVoroMap = True)
        #self._updatePathCostMap()
        
    def SetCell(self, s_cell: tuple, isVoroMap=False):
        current_map = self.distMap if(not isVoroMap) else self.voroDistMap
        current_map[s_cell[0]][s_cell[1]].nearest = s_cell
        current_map[s_cell[0]][s_cell[1]].dist = 0
        self.distOpen.put((0, s_cell)) if(not isVoroMap) else self.voroOpen.put((0, s_cell))
        current_map[s_cell[0]][s_cell[1]].toProcess = True
        
    def UnSetCell(self, s_cell: tuple, isVoroMap=False):
        current_map = self.distMap if(not isVoroMap) else self.voroDistMap
        current_map[s_cell[0]][s_cell[1]].nearest = self._unknown
        current_map[s_cell[0]][s_cell[1]].dist = float('inf')
        current_map[s_cell[0]][s_cell[1]].toRaise = True
        self.distOpen.put((0, s_cell)) if(not isVoroMap) else self.voroOpen.put((0, s_cell))
        current_map[s_cell[0]][s_cell[1]].toProcess = True
        
    def _updateMap(self, isVoroMap=False):
        if(not isVoroMap):
            current_queue = self.distOpen
            current_map = self.distMap
        else:
            current_queue = self.voroOpen
            current_map = self.voroDistMap
        while(current_queue.qsize()):
            current = current_queue.get()
            s_cell = current[1]
            if(not current_map[s_cell[0]][s_cell[1]].toProcess): continue
            if(current_map[s_cell[0]][s_cell[1]].toRaise):
                 self._raise(s_cell, current_map, isVoroMap)
            elif(self.isOcc(current_map[s_cell[0]][s_cell[1]].nearest, current_map)):
                if(not isVoroMap):
                    self.voro[s_cell[0]][s_cell[1]] = False
                    self.UnSetCell(s_cell, True)
                current_map[s_cell[0]][s_cell[1]].toProcess = False
                self._lower(s_cell, current_map, isVoroMap)
            
    def _raise(self, s_cell:tuple, current_map, isVoroMap):
        for n_cell in GetNeighbors(s_cell, (self.width, self.height)):
            if(current_map[n_cell[0]][n_cell[1]].nearest != self._unknown and 
               not current_map[n_cell[0]][n_cell[1]].toRaise):
                if(not self.isOcc(current_map[n_cell[0]][n_cell[1]].nearest, current_map)):
                    current_map[n_cell[0]][n_cell[1]].nearest = self._unknown
                    current_map[n_cell[0]][n_cell[1]].dist = float('inf')
                    current_map[n_cell[0]][n_cell[1]].toRaise = True
                
                self.distOpen.put((current_map[n_cell[0]][n_cell[1]].dist, n_cell)) if(not isVoroMap) else self.voroOpen.put((current_map[n_cell[0]][n_cell[1]].dist,n_cell))
                
                current_map[n_cell[0]][n_cell[1]].toProcess = True
        current_map[s_cell[0]][s_cell[1]].toRaise = False
        
    def _lower (self, s_cell:tuple, current_map, isVoroMap):
        for n_cell in GetNeighbors(s_cell, (self.width, self.height)):
            if(not current_map[n_cell[0]][n_cell[1]].toRaise):
                obst = current_map[s_cell[0]][s_cell[1]].nearest
                dx = obst[0] - n_cell[0]
                dy = obst[1] - n_cell[1]
                d = math.isqrt(dx*dx + dy*dy)
                if d < current_map[n_cell[0]][n_cell[1]].dist:
                    current_map[n_cell[0]][n_cell[1]].dist = d
                    current_map[n_cell[0]][n_cell[1]].nearest = current_map[s_cell[0]][s_cell[1]].nearest
                    
                    self.distOpen.put((d, n_cell)) if(not isVoroMap) else self.voroOpen.put((d,n_cell))
                    
                    current_map[n_cell[0]][n_cell[1]].toProcess = True
                elif(not isVoroMap):
                    self._chkVoro(s_cell, n_cell)   
                    
    def _chkVoro(self, s_cell:tuple, n_cell:tuple):
        if((self.distMap[s_cell[0]][s_cell[1]].dist > 1 or self.distMap[n_cell[0]][n_cell[1]].dist > 1)
          and self.distMap[n_cell[0]][n_cell[1]].nearest != self._unknown
          and self.distMap[n_cell[0]][n_cell[1]].nearest != self.distMap[s_cell[0]][s_cell[1]].nearest):
            
            obst = self.distMap[n_cell[0]][n_cell[1]].nearest
            dx =  s_cell[0] - obst[0]
            dy =  s_cell[1] - obst[1]
            dsN = math.isqrt(dx*dx + dy*dy)
            
            obst = self.distMap[s_cell[0]][s_cell[1]].nearest
            dx =  n_cell[0] - obst[0]
            dy =  n_cell[1] - obst[1]
            dnS = math.isqrt(dx*dx + dy*dy)
            
            sS = dsN - self.distMap[s_cell[0]][s_cell[1]].dist
            nS = dnS - self.distMap[n_cell[0]][n_cell[1]].dist
            
            if(sS <= nS):
                self.voro[s_cell[0]][s_cell[1]] = True
                self.SetCell(s_cell, True)
            if(nS <= sS):
                self.voro[n_cell[0]][n_cell[1]] = True
                self.SetCell(n_cell, True)
                
#     def _rebuildVoro(self, voroQ):
#         while(voroQ.qsize()):
#             s_cell = voroQ.get()
#             if(self._patternMatch(s_cell)): continue
            
            
#             self.voro[s_cell[0]][s_cell[1]] = False
#             for n_cell in self.GetNeighbors(s_cell):
#                 if(self.voro[n_cell[0]][n_cell[1]]):
#                     self.voroQ((self.dist[n_cell[0]][n_cell[1]], n_cell))
                    
#     def _patternMatch(self, s_cell:tuple):
#         return True
    
    def isOcc(self, cell:tuple, current_map):
        return cell != self._unknown and current_map[cell[0]][cell[1]].nearest == cell

