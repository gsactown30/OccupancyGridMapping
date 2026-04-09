import numpy as np
import open3d as o3d

def convertOrigin(pose):
    originX = (pose[0, 3] / 0.2).astype(int) + 200
    originY = (pose[1, 3] / 0.2).astype(int) + 200
    origin = (originX, originY)
    return origin

def convertCoordAll(scan, offset, pose=None, threeD = False):
    booleanMask = (np.abs(scan[:, 0]) < 30) & (np.abs(scan[:, 1]) < 30)
    newScan = scan[booleanMask].copy()

    if pose is not None:
        newScan[:, 3] = 1
        newScan = np.dot(newScan, pose.T)

    heightMask = (newScan[:, 2] < 2) & (newScan[:, 2] > -2)
    newScan = newScan[heightMask]

    groundMask = (newScan[:, 2] < -1.5) & (newScan[:, 2] > -2)
    groundPoints = newScan[groundMask][:, :3]
    pcdGround = o3d.geometry.PointCloud()
    pcdGround.points = o3d.utility.Vector3dVector(groundPoints)
    equation, _ = pcdGround.segment_plane(0.01, 3, 10)

    removalMask = np.abs(
        newScan[:, 0] * equation[0] +
        newScan[:, 1] * equation[1] +
        newScan[:, 2] * equation[2] +
        equation[3]
    ) > 0.1
    newScan = newScan[removalMask]

    coordX = (newScan[:, 0] / 0.2).astype(int) + offset
    coordY = (newScan[:, 1] / 0.2).astype(int) + offset

    if threeD:
        coordZ = (newScan[:, 2] / 0.2).astype(int) + 10
        return np.column_stack((coordX, coordY, coordZ))

    return np.column_stack((coordX, coordY))

class Grid():
    def __init__(self, xAxis, yAxis, zAxis = 1):
        self.array = np.zeros((zAxis, xAxis, yAxis))
        self.xAxis = xAxis
        self.yAxis = yAxis
        self.zAxis = zAxis
        self.center = (xAxis//2, yAxis//2)