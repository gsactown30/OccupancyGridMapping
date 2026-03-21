import numpy as np

def convertOrigin(pose):
    originX = (pose[0, 3] / 0.2).astype(int) + 200
    originY = (pose[1, 3] / 0.2).astype(int) + 200
    origin = (originX, originY)
    return origin

def convertCoordAll(scan, grid, pose):
    origin = convertOrigin(pose)
    centerX, centerY = origin
    booleanMask = (np.abs(scan[:, 0]) < 30) & (np.abs(scan[:, 1]) < 30) & (scan[:, 2] < 2)
    newScan = scan[booleanMask]
    newScan[:, 3] = 1
    newScan = np.dot(newScan, pose.T)
    coordX = (newScan[:, 0] / 0.2).astype(int) + 200
    coordY = (newScan[:, 1] / 0.2).astype(int) + 200
    return np.column_stack((coordX, coordY))

class Grid():
    def __init__(self, xAxis, yAxis):
        self.array = np.zeros((xAxis, yAxis))
        self.xAxis = xAxis
        self.yAxis = yAxis
        self.center = (xAxis//2, yAxis//2)