import numpy
import numpy as np
import math

#overall i think logic is sound but not sure if implementation is correct for logodds updates
#also not sure where center should be/ what the outer bounds of the grid is

def convertCoord(x, y, grid):
    centerX, centerY = grid.center
    gridIndex = (int(x / 0.2) + centerX, int(y / 0.2) + centerY)
    return gridIndex

def convertProb(odd):
    return 1/(1+math.e**-odd)

class Grid():
    def __init__(self, xAxis, yAxis):
        self.array = np.zeros((xAxis, yAxis))
        self.xAxis = xAxis
        self.yAxis = yAxis
        self.center = (xAxis//2, yAxis//2)

    def updateValue(self, x, y, sensorP):
        '''p = self.array[x,y]
        p = np.log(p/(1-p))
        pNew = p + np.log(sensorP/(1-sensorP))
        self.array[x,y] = pNew'''
        self.array[x, y] += sensorP
        self.array[x, y] = max(-100, min(self.array[x, y], 100))
        #self.array = np.clip(self.array, -100, 100)




