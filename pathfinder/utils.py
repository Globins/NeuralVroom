import numpy as np

#These functions are general functions can be used in mult. files

def cart2pol(x, y):
    """returns polar coords of point (x, y) in relation to (0,0)"""
    r = np.sqrt(x*x + y*y)
    theta = np.arctan2(y, x)
    return r, theta

def mod2Pi(theta):
    """ returns angle phi = theta%2pi with contraint -pi <= theta < pi"""
    theta = theta % (2*np.pi)
    if theta < -np.pi: return theta + 2*np.pi
    if theta >= np.pi: return theta - 2*np.pi
    return theta

def change_of_basis(start:tuple, end:tuple):
    """translates the end point to be in start's coordinate system with start being (0,0)
    start and end are passed in(x, y, orientation)"""
    theta1 = np.deg2rad(start[2])
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    new_x = dx * np.cos(theta1) + dy*np.sin(theta1)
    new_y = -dx * np.sin(theta1) + dy*np.cos(theta1)
    new_theta = end[2] - start[2]
    return new_x, new_y, new_theta

def IsPointInGrid(point:tuple = (-1, -1), gridDimensions:tuple = (-1, -1)):
    """determines if the point is inside the grid"""
    return (point[0] >= 0 and point[1] >= 0) and (point[0] < gridDimensions[0] and point[1] < gridDimensions[1])

def GetNeighbors(cell:tuple, gridDimensions:tuple):
    """Gets all surrounding neighbors"""
    x = cell[0]
    y = cell[1]
    neighbors = []
    if(not IsPointInGrid(cell, gridDimensions)):
        return neighbors
    if(x-1 >= 0):
        neighbors.append((x-1,y))
        if(y-1 >= 0): neighbors.append((x-1,y-1))
        if(y+1 < gridDimensions[1]): neighbors.append((x-1,y+1))
        pass
    if(x+1 < gridDimensions[0]):
        neighbors.append((x+1,y))
        if(y-1 >= 0): neighbors.append((x+1,y-1))
        if(y+1 < gridDimensions[1]): neighbors.append((x+1,y+1))
        pass
    if(y-1 >= 0): neighbors.append((x,y-1))
    if(y+1 < gridDimensions[1]): neighbors.append((x,y+1))
    return neighbors

def rasterizeline(start:tuple, end:tuple, gridDimensions:tuple):
    """Returns a list of cells from start to end"""
    x0 = start[0]
    y0 = start[1]
    x1 = end[0]
    y1 = end[1]
    line = []
    isSteep = abs(y1-y0) > abs(x1-x0)
    if(isSteep):
        temp = x0
        x0 = y0
        y0 = temp

        temp = x1
        x1 = y1
        y1 = temp
    if(x0 > x1):
        temp = x0
        x0 = x1
        x1 = temp

        temp = y0
        y0 = y1
        y1 = temp
    dx = x1 - x0
    dy = abs(y1-y0)
    err = dx /2
    ystep = 1 if y0 < y1 else -1
    y = y0
    for x in range(x0,x1+1):
        cell = (x,y)
        if(isSteep):
            cell = (y,x)
        if(IsPointInGrid(cell, gridDimensions)):
            line.append(cell)
        err -= dy
        if(err < 0):
            y += ystep
            err += dx
    return line

def rotate(xy, radians):
    """dot product of position and orientation"""
    x, y = xy
    c, s = np.cos(radians), np.sin(radians)
    j = np.matrix([[c, -s], [s, c]])
    m = np.dot(j, [x, y])
    
    finalx = float(m.T[0])
    finaly = float(m.T[1])
    return finalx, finaly

def contToCell(self):
    """Converts the continuous value into a discrete-friendly value"""
    pass

def rpoint(x, y, r, deg):
    """ Get a point on a radius at deg degrees."""
    rang = np.deg2rad(deg)
    x1 = x + r * np.cos(rang)
    y1 = y + r * np.sin(rang)
    return (x1, y1)
