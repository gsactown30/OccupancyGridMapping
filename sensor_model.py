import pykitti
import numpy as np
from grid import Grid
from grid import convertCoordAll2D
from grid import convertCoordAll3D
from grid import convertCoordAllAnim
from grid import convertOrigin
from raycast import bresenham
import time
import open3d as o3d
import matplotlib.pyplot as plt
import imageio

startTime = time.perf_counter()

basedir = './data/kitti'
date = '2011_09_26'
drive = '0015'
dataset = pykitti.raw(basedir, date, drive)

choice = int(input('2d(1) or 3d(2) scan or 2d animation(3)'))
startScan = int(input('starting scan (0 for default)'))
maxScan = startScan + int(input('how many scans'))

if choice == 1:
    occupancyGrid = Grid(2000, 2000)

    for i in range(startScan, maxScan):
        scan = dataset.get_velo(i)
        pose = dataset.oxts[i].T_w_imu
        newScan = convertCoordAll2D(scan, pose)
        origin = convertOrigin(pose)

        arrFree = []
        arrHit = []

        for point in newScan:
            coord = (point[0], point[1])
            arrFree.extend(bresenham(origin, coord))
            arrHit.append(coord)

        arrFree = np.array(arrFree)
        np.add.at(occupancyGrid.array, (0, arrFree[:, 0], arrFree[:, 1]), -0.2)

        arrHit = np.array(arrHit)
        np.add.at(occupancyGrid.array, (0, arrHit[:, 0], arrHit[:, 1]), 2)
        occupancyGrid.array = np.clip(occupancyGrid.array, -10, 10)

    probabilities = 1 / (1 + np.exp(-(occupancyGrid.array)))
    probabilities = np.sum(probabilities, 0)

    plt.imshow(probabilities, cmap='binary', interpolation='nearest', vmax=1, vmin=0, origin="lower")
    plt.colorbar()
    plt.savefig("testGrid")

    endTime = time.perf_counter()
    print(f"Program runtime: {endTime - startTime:.2f} seconds")

    plt.show()

elif choice == 2:
    occupancyGrid = Grid(2000, 2000, 20)

    for i in range(startScan, maxScan):
        scan = dataset.get_velo(i)
        pose = dataset.oxts[i].T_w_imu
        newScan = convertCoordAll3D(scan, pose)
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
        np.add.at(occupancyGrid.array, (arrHit[:, 2], arrHit[:, 0], arrHit[:, 1]), 2)
        occupancyGrid.array = np.clip(occupancyGrid.array, -10, 10)

    probabilities = 1 / (1 + np.exp(-(occupancyGrid.array)))
    occupied = np.where(probabilities > 0.6)
    ocZ, ocX, ocY = occupied

    ocZ = (ocZ - 10) * 0.2
    ocX = (ocX - 200) * 0.2
    ocY = (ocY - 200) * 0.2
    newOccupied = np.column_stack((ocX, ocY, ocZ))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(newOccupied)
    #o3d.geometry.PointCloud.segment_plane()
    pcd.remove_statistical_outlier(20, 2, True)

    endTime = time.perf_counter()
    print(f"Program runtime: {endTime - startTime:.2f} seconds")

    o3d.visualization.draw_geometries([pcd])

elif choice == 3:
    occupancyGrid = Grid(301, 301)
    frames = []

    for i in range(startScan, maxScan):
        scan = dataset.get_velo(i)
        newScan = convertCoordAllAnim(scan)
        origin = occupancyGrid.center

        arrFree = []
        arrHit = []

        for point in newScan:
            coord = (point[0], point[1])
            arrFree.extend(bresenham(origin, coord))
            arrHit.append(coord)

        arrFree = np.array(arrFree)
        np.add.at(occupancyGrid.array, (0, arrFree[:, 0], arrFree[:, 1]), -0.2)

        arrHit = np.array(arrHit)
        np.add.at(occupancyGrid.array, (0, arrHit[:, 0], arrHit[:, 1]), 2)
        occupancyGrid.array = np.clip(occupancyGrid.array, -10, 10)

        probabilities = 1 / (1 + np.exp(-(occupancyGrid.array)))

        frames.append((probabilities[0] * 255).astype(np.uint8))

    imageio.mimsave("occupancy.gif", frames, fps=10)

    endTime = time.perf_counter()
    print(f"Program runtime: {endTime - startTime:.2f} seconds")