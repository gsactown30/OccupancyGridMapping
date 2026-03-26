import numpy as np

def convertOrigin(pose):
    originX = (pose[0, 3] / 0.2).astype(int) + 200
    originY = (pose[1, 3] / 0.2).astype(int) + 200
    #originZ = (pose[2, 3] / 0.2).astype(int) + 10
    origin = (originX, originY)
    return origin

def convertCoordAll(scan, pose):
    booleanMask = (np.abs(scan[:, 0]) < 30) & (np.abs(scan[:, 1]) < 30)
    newScan = scan[booleanMask]
    newScan[:, 3] = 1
    newScan = np.dot(newScan, pose.T)
    heightMask = (newScan[:, 2] < 2) & (newScan[:, 2] > -2)
    newScan = newScan[heightMask]
    coordX = (newScan[:, 0] / 0.2).astype(int) + 200
    coordY = (newScan[:, 1] / 0.2).astype(int) + 200
    coordZ = (newScan[:, 2] / 0.2).astype(int) + 10
    return np.column_stack((coordX, coordY, coordZ))

class Grid():
    def __init__(self, xAxis, yAxis, zAxis):
        self.array = np.zeros((zAxis, xAxis, yAxis))
        self.xAxis = xAxis
        self.yAxis = yAxis
        self.zAxis = zAxis
        #self.center = (xAxis//2, yAxis//2)