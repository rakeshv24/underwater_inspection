import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

import GenerateClouds
import SliceCloud

originX, originY, originZ = 50, 50, 50
sphereRadius = 5
points = GenerateClouds.generateUniformCloud(originX, originY, originZ, sphereRadius, numPoints=10000)

pitch = math.pi/4
yaw = math.pi/4
capRadius = 2
sectionPoints = SliceCloud.sliceSphericalCap(points, originX, originY, originZ, sphereRadius, pitch, yaw, capRadius)
# sectionPoints, translatedPoints = SliceCloud.slicePatch(points, math.pi/4, math.pi/4, 2, 1, 2)

# Matplotlib Sample Code using 2D arrays via meshgrid
X = points[:, 0]
Y = points[:, 1]
Z = points[:, 2]

X2 = sectionPoints[:, 0]
Y2 = sectionPoints[:, 1]
Z2 = sectionPoints[:, 2]

fig = plt.figure()
ax = Axes3D(fig)
# surf = ax.scatter3D(X, Y, Z)
surf2 = ax.scatter3D(X2, Y2, Z2)

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_aspect('auto')
plt.show()