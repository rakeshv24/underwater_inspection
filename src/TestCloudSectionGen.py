import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

import GenerateClouds
import SliceCloud

originX, originY, originZ = 0, 0, 0
sr1, sr2, sr3 = 2.5, 5, 7.5
pitch = 0
yaw = 0
capRadius = 2

points1 = GenerateClouds.generateUniformCloud(originX, originY, originZ, sr1, numPoints=10000)
sectionPoints1 = SliceCloud.sliceSphericalCap(points1, originX, originY, originZ, sr1, pitch, yaw, capRadius)

points2 = GenerateClouds.generateUniformCloud(originX, originY, originZ, sr2, numPoints=10000)
sectionPoints2 = SliceCloud.sliceSphericalCap(points2, originX, originY, originZ, sr2, pitch, yaw, capRadius)

points3 = GenerateClouds.generateUniformCloud(originX, originY, originZ, sr3, numPoints=10000)
sectionPoints3 = SliceCloud.sliceSphericalCap(points3, originX, originY, originZ, sr3, pitch, yaw, capRadius)

# sectionPoints, translatedPoints = SliceCloud.slicePatch(points, math.pi/4, math.pi/4, 2, 1, 2)

# Matplotlib Sample Code using 2D arrays via meshgrid
# x = points[:, 0]
# Y = points[:, 1]
# Z = points[:, 2]

xSP1 = sectionPoints1[:, 0]
ySP1 = sectionPoints1[:, 1]
zSP1 = sectionPoints1[:, 2]

xSP2 = sectionPoints2[:, 0]
ySP2 = sectionPoints2[:, 1]
zSP2 = sectionPoints2[:, 2]

xSP3 = sectionPoints3[:, 0]
ySP3 = sectionPoints3[:, 1]
zSP3 = sectionPoints3[:, 2]

fig = plt.figure()
ax = Axes3D(fig)
# surf = ax.scatter3D(X, Y, Z)
sec1 = ax.scatter3D(xSP1, ySP1, zSP1, color='red')
sec2 = ax.scatter3D(xSP2, ySP2, zSP2, color='blue')
sec3 = ax.scatter3D(xSP3, ySP3, zSP3, color='green')

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

ax.set_xlim(-8, 8)
ax.set_ylim(-8, 8)
ax.set_zlim(-8, 8)

plt.show()