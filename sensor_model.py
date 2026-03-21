import pykitti
import numpy as np
from grid import Grid
from grid import convertCoordAll
from raycast import bresenham
import matplotlib.pyplot as plt

basedir = './data/kitti'
date = '2011_09_26'
drive = '0015'

dataset = pykitti.raw(basedir, date, drive)
occupancyGrid = Grid(301, 301)

for i in range(0, 1):
    scan = dataset.get_velo(i)
    newScan = convertCoordAll(scan, occupancyGrid)

    arrFree = []
    arrHit = []

    for point in newScan:
        origin = occupancyGrid.center
        coord = (point[0], point[1])
        arrFree.extend(bresenham(origin, coord))
        arrHit.append(coord)

    arrFree = np.array(arrFree)
    np.add.at(occupancyGrid.array, (arrFree[:, 0], arrFree[:, 1]), -0.4)

    arrHit = np.array(arrHit)
    np.add.at(occupancyGrid.array, (arrHit[:, 0], arrHit[:, 1]), 0.85)

occupancyGrid.array = np.clip(occupancyGrid.array, -100, 100)
probabilities = 1 / (1 + np.exp(-(occupancyGrid.array)))

plt.imshow(probabilities, cmap='binary', interpolation='nearest', vmax=1, vmin=0, origin="lower")
plt.colorbar()
plt.savefig("testGrid")
plt.show()