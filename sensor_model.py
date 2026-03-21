import pykitti
import numpy as np
from grid import Grid
from grid import convertCoordAll
from grid import convertOrigin
from raycast import bresenham
import matplotlib.pyplot as plt
import time

startTime = time.perf_counter()

basedir = './data/kitti'
date = '2011_09_26'
drive = '0015'

dataset = pykitti.raw(basedir, date, drive)
occupancyGrid = Grid(2000, 2000)

for i in range(0, 1):
    scan = dataset.get_velo(i)
    pose = dataset.oxts[i].T_w_imu
    newScan = convertCoordAll(scan, pose)
    origin = convertOrigin(pose)

    arrFree = []
    arrHit = []

    for point in newScan:
        coord = (point[0], point[1])
        arrFree.extend(bresenham(origin, coord))
        arrHit.append(coord)

    arrFree = np.array(arrFree)
    np.add.at(occupancyGrid.array, (arrFree[:, 0], arrFree[:, 1]), -0.2)

    arrHit = np.array(arrHit)
    np.add.at(occupancyGrid.array, (arrHit[:, 0], arrHit[:, 1]), 1)
    occupancyGrid.array = np.clip(occupancyGrid.array, -10, 10)

probabilities = 1 / (1 + np.exp(-(occupancyGrid.array)))

plt.imshow(probabilities, cmap='binary', interpolation='nearest', vmax=1, vmin=0, origin="lower")
plt.colorbar()
plt.savefig("testGrid")

endTime = time.perf_counter()
print(f"Program runtime: {endTime - startTime:.2f} seconds")

plt.show()