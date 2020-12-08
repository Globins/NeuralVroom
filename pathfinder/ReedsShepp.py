from .vehicle import *
from .utils import *

class ReedsSheppAction:
    """Represents a single action within a path from point A to B"""
    def __init__(self, steer, gear, length):
        self.Steer = steer
        self.Gear = gear
        self.Length = length
        
    def RevSteer(self):
        if(self.Steer == Steer.Left):
            self.Steer = Steer.Right
        elif(self.Steer == Steer.Right):
            self.Steer = Steer.Left
    
    def RevGear(self):
        self.Gear = Gear.Backward if(self.Gear == Gear.Forward) else Gear.Forward
    def __repr__(self):
         return 'Action(Steer=%s, Gear=%s, Distance=%s)' % (self.Steer, self.Gear, self.Length)

#Single Steering and motion over some length FORMULAS BASED OFF THE REEDS SHEPP PAPER:
#https://projecteuclid.org/download/pdf_1/euclid.pjm/1102645450
#some equations are modified using optimizations from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
class ReedsShepp:
    def __init__(self, vehicle:Vehicle):
        self._vehicle = vehicle
    
    def calculatePathLength(self, path):
        return sum([segment.Length for segment in path])
    
    def run_optimalpath(self, start, end):
        paths = self.run_allpaths(start, end)
        index = 0
        smallestLength = self.calculatePathLength(paths[0])
        for i in range(1, len(paths)-1):
            length = self.calculatePathLength(paths[i])
            if length <= smallestLength:
                smallestLength, index = length, i
        return paths[index]
    
    def run_allpaths(self, start, end):
        paths = []
        path_funcs = [self._CSC_SameTurns, self._CSC_DiffTurns, self._C_C_C, self._C_CC, self._CC_C, 
                      self._CCu_CuC, self._C_CuCu_C, self._C_Cpi2SC_SameTurn, self._C_Cpi2SC_DiffTurn,
                      self._CSCp2_C_SameTurn, self._CSCp2_C_DiffTurn, self._C_Cpi2SCpi2_C]
        x, y, phi = change_of_basis(start,end)
        for path in path_funcs:
            paths.append(path(x,y,phi))
            paths.append(self._reverseGearOfPath(path(-x,y,-phi)))
            paths.append(self._reverseSteerOfPath(path(x,-y,-phi)))
            paths.append(self._reverseSteerOfPath(self._reverseGearOfPath(path(-x,-y,phi))))
        
        for i in range(len(paths)):
            paths[i] = list(filter(lambda e: e.Length != 0, paths[i]))

        paths = list(filter(None, paths))
        return paths

    def _reverseSteerOfPath(self, path):
        newPath = path.copy()
        for segment in newPath:
            segment.RevSteer()
        return newPath
    
    def _reverseGearOfPath(self,path):
        newPath = path.copy()
        for segment in newPath:
            segment.RevGear()
        return newPath
    
    def _CSC_SameTurns(self,x, y, phi): #Same Turns
        segment = []
        phi = np.deg2rad(phi)

        u, t = cart2pol(x - np.sin(phi), y - 1 + np.cos(phi))
        v = mod2Pi(phi - t)
        if t >= 0 and u >= 0 and v >= 0:
            segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
            segment.append(ReedsSheppAction(Steer.Straight, Gear.Forward, u))
            segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, v))
        return segment
        
    def _CSC_DiffTurns(self,x, y, phi): #csc opposite turns
        segment = []
        phi = mod2Pi(np.deg2rad(phi))
        r, theta = cart2pol(x + np.sin(phi), y - 1 - np.cos(phi))
        if (r*r >= 4):
            u = np.sqrt(r*r - 4)
            t = mod2Pi(theta +np.arctan2(2, u))
            v = mod2Pi(t - phi)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Straight, Gear.Forward, u))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Forward, v))  
        return segment
    
    def _C_C_C(self,x, y, phi): #C|C|C
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x - np.sin(phi), y - 1 + np.cos(phi))

        if (r <= 4):
            A = np.arccos(r/4)
            t = mod2Pi(theta + np.pi/2 + A)
            u = mod2Pi(np.pi - 2*A)
            v = mod2Pi(phi - t - u)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, v))
        return segment
    
    def _C_CC(self,x, y, phi): # C|CC
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x - np.sin(phi), y - 1 + np.cos(phi))
        if (r <= 4):
            A = np.arccos(r/4)
            t = mod2Pi(theta + np.pi/2 + A)
            u = mod2Pi(np.pi - 2*A)
            v = mod2Pi(t + u - phi)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Backward, v))
        return segment
    
    def _CC_C(self,x, y, phi): # CC|C
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x - np.sin(phi), y - 1 + np.cos(phi))
        if (r <= 4):
            u = np.arccos(1 - r*r/8)
            A = np.arcsin(2*np.sin(u)/r)
            t = mod2Pi(theta + np.pi/2 - A)
            v = mod2Pi(t - u - phi)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Forward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Backward, v))
        return segment
    
    def _CCu_CuC(self,x, y, phi): #CCu|CuC
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x + np.sin(phi), y - 1 - np.cos(phi))
        if (r <= 4):
            if(r <= 2):
                A = np.arccos((r+2)/4)
                t = mod2Pi(theta + np.pi/2 + A)
                u = mod2Pi(A)
                v = mod2Pi(phi - t + 2*u)
            else:
                A = np.arccos((r-2)/4)
                t = mod2Pi(theta + np.pi/2 - A)
                u = mod2Pi(np.pi - A)
                v = mod2Pi(phi - t + 2*u)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Forward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, v))
        return segment
    
    def _C_CuCu_C(self,x, y, phi): #C|CuCu|C
        segment = []
        r, theta = cart2pol(x ,y)
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x + np.sin(phi), y - 1 - np.cos(phi))
        u1 = (20 - r*r) /16
        if (r <= 6 and 0 <= u1 and u1 <= 1):
            u = np.arccos(u1)
            A = np.arcsin(2*np.sin(u)/r)
            t = mod2Pi(theta + np.pi/2 + A)
            v = mod2Pi(t - phi)
            
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Forward, v))
        return segment
    
    def _C_Cpi2SC_SameTurn(self,x, y, phi): #C|C(pi/2)SC same turn 8
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x - np.sin(phi), y - 1 + np.cos(phi))
        if(r >= 2):
            u = np.sqrt(r*r - 4) - 2
            A = np.arctan2(2, u+2)
            t = mod2Pi(theta + np.pi/2 + A)
            v = mod2Pi(t - phi + np.pi/2)
            
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, np.pi/2))
                segment.append(ReedsSheppAction(Steer.Straight, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Backward, v))
        return segment
    
    def _C_Cpi2SC_DiffTurn(self,x, y, phi): #C|C(pi/2)SC differnet turn 9
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x + np.sin(phi), y - 1 - np.cos(phi))
        if(r >= 2):
            t = mod2Pi(theta + np.pi/2)
            u = r - 2
            v = mod2Pi(phi - t - np.pi/2)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, np.pi/2))
                segment.append(ReedsSheppAction(Steer.Straight, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, v))
        return segment

    
    def _CSCp2_C_SameTurn(self,x, y, phi): #CSC(pi/2)|C same turn 10
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x - np.sin(phi), y - 1 + np.cos(phi))
        if(r >= 2):
            u = np.sqrt(r*r - 4) - 2
            A = np.arctan2(u+2, 2)
            t = mod2Pi(theta + np.pi/2 - A)
            v = mod2Pi(t - phi - np.pi/2)
            
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Straight, Gear.Forward, u))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Forward, np.pi/2))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Backward, v))
        return segment
    
    def _CSCp2_C_DiffTurn(self,x, y, phi): #CSC(pi/2)|C diffrent turn
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x + np.sin(phi), y - 1 - np.cos(phi))
        if(r >= 2):
            t = mod2Pi(theta)
            u = r - 2
            v = mod2Pi(phi - t - np.pi/2)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Straight, Gear.Forward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, np.pi/2))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, v))
        return segment
    
    def _C_Cpi2SCpi2_C(self,x,y,phi): #C|C(pi/2)SC(pi/2)|c
        segment = []
        phi = np.deg2rad(phi)
        r, theta = cart2pol(x + np.sin(phi), y - 1 - np.cos(phi))
        if(r >= 4):
            u = np.sqrt(r*r - 4) - 4
            A = np.arctan2(2, u+4)
            t = mod2Pi(theta + np.pi/2 + A)
            v = mod2Pi(t - phi)
            if t >= 0 and u >= 0 and v >= 0:
                segment.append(ReedsSheppAction(Steer.Left, Gear.Forward, t))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Backward, np.pi/2))
                segment.append(ReedsSheppAction(Steer.Straight, Gear.Backward, u))
                segment.append(ReedsSheppAction(Steer.Left, Gear.Backward, np.pi/2))
                segment.append(ReedsSheppAction(Steer.Right, Gear.Forward, v))
        return segment
        
    def discretizePath(self, start:VehicleState, actions, unit, length):
        """Returns discretized vehiclestates"""
        current = start
        states = []
        states.append(current)
        for action in actions:
            n = int(np.ceil(action.Length * unit / length))
            if(action.Steer != Steer.Straight):
                angle = action.Length / n
                dx = unit*np.sin(angle)
                dy = unit-unit*np.cos(angle)
                if(action.Steer == Steer.Right):
                    dy = -dy
                    angle = -angle
                if(action.Gear == Gear.Backward):
                    dx = -dx
                    angle = -angle
                for i in range(int(n)):
                    rotated = rotate((dx,dy), current.orientation)
                    current = VehicleState(tuple(np.add(rotated, current.position)), current.orientation + angle, action.Gear)
                    states.append(current)
            else:
                pl = action.Length * unit / n
                dx = pl * np.cos(current.orientation)
                dy = pl * np.sin(current.orientation)
                if(action.Gear == Gear.Backward):
                    dx = -dx
                    dy = -dy
                for i in range(int(n)):
                    current = VehicleState((dx + current.position[0], dy + current.position[1]), current.orientation, action.Gear)
                    states.append(current)
        return states