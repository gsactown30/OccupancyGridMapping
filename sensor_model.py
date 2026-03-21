import pykitti
import numpy as np
import sys
from grid import convertCoord
from grid import Grid
from grid import convertProb
from raycast import bresenham
import matplotlib.pyplot as plt

basedir = './data/kitti'
date = '2011_09_26'
drive = '0015'

dataset = pykitti.raw(basedir, date, drive)

scan = dataset.get_velo(0)

occupancyGrid = Grid(301, 301)
#'''
for point in scan:
    #checks if points are atleast below 2 meters or above the ground
    #checks if points are within 30 meters (gave arbitrary number for now)
    if abs(point[0]) > 30 or abs(point[1]) > 30 or point[2] < -2 or point[2] > 2:
        continue
    else:
        #converted hit point to grid coord
        coord = convertCoord(point[0], point[1], occupancyGrid)
        #arbitrarly choose 0,0 as car/sensor center not sure what actual center is
        #do I have to do like for all in scan check max x and y then divide by 2?
        origin = occupancyGrid.center
        arr = bresenham(origin, coord)
        #updates all points up till hit cell to update with 0?
        for hit in arr:
            hitX, hitY = hit
            occupancyGrid.updateValue(hitX, hitY, -0.4)
        #wasnt sure if bresenham would include the endpoint in arr so manually just reupdated end-
        #point just to be safe so it doesnt accidentally get updated as not hit
        coordX, coordY = coord
        occupancyGrid.updateValue(coordX, coordY, 0.85)

#'''

probabilities = 1 / (1 + np.exp(-(occupancyGrid.array)))

plt.imshow(probabilities, cmap='binary', interpolation='nearest', vmax=1, vmin=0, origin="lower")
plt.colorbar()
plt.savefig("testGrid")
plt.show()

print(f"Max x: {scan[:,0].max():.1f}  Min x: {scan[:,0].min():.1f}")
print(f"Max y: {scan[:,1].max():.1f}  Min y: {scan[:,1].min():.1f}")
print(f"Max z: {scan[:,2].max():.1f}  Min z: {scan[:,2].min():.1f}")
#with np.printoptions(threshold=sys.maxsize):
print(scan)
print(scan[0])
print(scan[123838])
print(len(scan))
print(occupancyGrid.array.max())
print(occupancyGrid.array.min())