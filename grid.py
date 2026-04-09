import numpy as np
import open3d as o3d

def convertOrigin(pose):
    originX = (pose[0, 3] / 0.2).astype(int) + 200
    originY = (pose[1, 3] / 0.2).astype(int) + 200
    origin = (originX, originY)
    return origin

def convertCoordAll3D(scan, pose):
    booleanMask = (np.abs(scan[:, 0]) < 30) & (np.abs(scan[:, 1]) < 30)
    newScan = scan[booleanMask]
    newScan[:, 3] = 1
    newScan = np.dot(newScan, pose.T)
    heightMask = (newScan[:, 2] < 2) & (newScan[:, 2] > -2)
    newScan = newScan[heightMask]

    groundMask = (newScan[:, 2] < -1.5) & (newScan[:, 2] > -2)
    groundPoints = newScan[groundMask]
    groundPoints = groundPoints[:, :3]
    pcdGround = o3d.geometry.PointCloud()
    pcdGround.points = o3d.utility.Vector3dVector(groundPoints)
    groundPlane = pcdGround.segment_plane(0.01, 3, 10)
    equation, pointsInPlane = groundPlane
    removalMask = np.abs((newScan[:, 0] * equation[0]) + (newScan[:, 1] * equation[1]) + (newScan[:, 2] * equation[2]) + (equation[3])) > 0.1
    newScan = newScan[removalMask]

    coordX = (newScan[:, 0] / 0.2).astype(int) + 200
    coordY = (newScan[:, 1] / 0.2).astype(int) + 200
    coordZ = (newScan[:, 2] / 0.2).astype(int) + 10
    return np.column_stack((coordX, coordY, coordZ))

def convertCoordAll2D(scan, pose):
    booleanMask = (np.abs(scan[:, 0]) < 30) & (np.abs(scan[:, 1]) < 30)
    newScan = scan[booleanMask]
    newScan[:, 3] = 1
    newScan = np.dot(newScan, pose.T)
    heightMask = (newScan[:, 2] < 2) & (newScan[:, 2] > -2)
    newScan = newScan[heightMask]

    groundMask = (newScan[:, 2] < -1.5) & (newScan[:, 2] > -2)
    groundPoints = newScan[groundMask]
    groundPoints = groundPoints[:, :3]
    pcdGround = o3d.geometry.PointCloud()
    pcdGround.points = o3d.utility.Vector3dVector(groundPoints)
    groundPlane = pcdGround.segment_plane(0.01, 3, 10)
    equation, pointsInPlane = groundPlane
    removalMask = np.abs((newScan[:, 0] * equation[0]) + (newScan[:, 1] * equation[1]) + (newScan[:, 2] * equation[2]) + (equation[3])) > 0.1
    newScan = newScan[removalMask]

    coordX = (newScan[:, 0] / 0.2).astype(int) + 200
    coordY = (newScan[:, 1] / 0.2).astype(int) + 200
    return np.column_stack((coordX, coordY))

def convertCoordAllAnim(scan):
    booleanMask = (np.abs(scan[:, 0]) < 30) & (np.abs(scan[:, 1]) < 30)
    newScan = scan[booleanMask]
    heightMask = (newScan[:, 2] < 2) & (newScan[:, 2] > -2)
    newScan = newScan[heightMask]

    groundMask = (newScan[:, 2] < -1.5) & (newScan[:, 2] > -2)
    groundPoints = newScan[groundMask]
    groundPoints = groundPoints[:, :3]
    pcdGround = o3d.geometry.PointCloud()
    pcdGround.points = o3d.utility.Vector3dVector(groundPoints)
    groundPlane = pcdGround.segment_plane(0.01, 3, 10)
    equation, pointsInPlane = groundPlane
    removalMask = np.abs((newScan[:, 0] * equation[0]) + (newScan[:, 1] * equation[1]) + (newScan[:, 2] * equation[2]) + (equation[3])) > 0.1
    newScan = newScan[removalMask]

    coordX = (newScan[:, 0] / 0.2).astype(int) + 150
    coordY = (newScan[:, 1] / 0.2).astype(int) + 150
    return np.column_stack((coordX, coordY))

class Grid():
    def __init__(self, xAxis, yAxis, zAxis = 1):
        self.array = np.zeros((zAxis, xAxis, yAxis))
        self.xAxis = xAxis
        self.yAxis = yAxis
        self.zAxis = zAxis
        self.center = (xAxis//2, yAxis//2)