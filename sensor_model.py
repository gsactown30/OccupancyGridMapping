import pykitti
import numpy as np
from grid import Grid
from grid import convertCoordAll
from grid import convertOrigin
from raycast import bresenham
import matplotlib.pyplot as plt
import time
import open3d as o3d

startTime = time.perf_counter()

basedir = './data/kitti'
date = '2011_09_26'
drive = '0015'

dataset = pykitti.raw(basedir, date, drive)
occupancyGrid = Grid(4000, 4000, 50)
scan = dataset.get_velo(165)
print(f"Max x: {scan[:,0].max():.1f}  Min x: {scan[:,0].min():.1f}")
print(f"Max y: {scan[:,1].max():.1f}  Min y: {scan[:,1].min():.1f}")
print(f"Max z: {scan[:,2].max():.1f}  Min z: {scan[:,2].min():.1f}")

for i in range(0, 1):
    scan = dataset.get_velo(i)
    pose = dataset.oxts[i].T_w_imu
    newScan = convertCoordAll(scan, pose)
    origin = convertOrigin(pose)

    arrFree = []
    arrHit = []

    for point in newScan:
        coord = (point[0], point[1])
        coordHit = (point[0], point[1], point[2])
        arrFree.extend(bresenham(origin, coord))
        arrHit.append(coordHit)

    arrFree = np.array(arrFree)
    np.add.at(occupancyGrid.array, (slice(None), arrFree[:, 0], arrFree[:, 1]), -0.2)

    arrHit = np.array(arrHit)
    np.add.at(occupancyGrid.array, (arrHit[:, 2], arrHit[:, 0], arrHit[:, 1]), 1)
    occupancyGrid.array = np.clip(occupancyGrid.array, -10, 10)

probabilities = 1 / (1 + np.exp(-(occupancyGrid.array)))
occupied = np.where(probabilities > 0.6)
ocZ, ocX, ocY = occupied

ocZ = (ocZ - 25) * 0.2
ocX = (ocX - 1000) * 0.2
ocY = (ocY - 1000) * 0.2
newOccupied = np.column_stack((ocX, ocY, ocZ))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(newOccupied)
#o3d.geometry.PointCloud.segment_plane()
o3d.visualization.draw_geometries([pcd])

print(occupancyGrid.array.max())
print(occupancyGrid.array.min())
print(np.count_nonzero(occupancyGrid.array))
'''
plt.imshow(probabilities, cmap='binary', interpolation='nearest', vmax=1, vmin=0, origin="lower")
plt.colorbar()
plt.savefig("testGrid")
'''

endTime = time.perf_counter()
print(f"Program runtime: {endTime - startTime:.2f} seconds")

#plt.show()