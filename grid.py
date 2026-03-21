import numpy as np

def convertCoordAll(scan, grid):
    centerX, centerY = grid.center
    booleanMask = (np.abs(scan[:, 0]) < 30) & (np.abs(scan[:, 1]) < 30) & (scan[:, 2] > -2) & (scan[:, 2] < 2)
    newScan = scan[booleanMask]
    coordX = (newScan[:, 0] / 0.2).astype(int) + centerX
    coordY = (newScan[:, 1] / 0.2).astype(int) + centerY
    return np.column_stack((coordX, coordY))

class Grid():
    def __init__(self, xAxis, yAxis):
        self.array = np.zeros((xAxis, yAxis))
        self.xAxis = xAxis
        self.yAxis = yAxis
        self.center = (xAxis//2, yAxis//2)