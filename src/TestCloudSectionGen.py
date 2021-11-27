import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

import GenerateClouds
import SliceCloud

points = GenerateClouds.generateUniformCloud(0, 0, 0, 2, numPoints=10000)
# sectionPoints = SliceCloud.sliceSphericalCap(points, math.pi/4, math.pi, 1)
sectionPoints, translatedPoints = SliceCloud.slicePatch(points, math.pi/4, math.pi/4, 2, 1, 2)

# Matplotlib Sample Code using 2D arrays via meshgrid
X = sectionPoints[:, 0]
Y = sectionPoints[:, 1]
Z = sectionPoints[:, 2]

X2 = translatedPoints[:, 0]
Y2 = translatedPoints[:, 1]
Z2 = translatedPoints[:, 2]

fig = plt.figure()
ax = Axes3D(fig)
surf = ax.scatter3D(X, Y, Z)
surf2 = ax.scatter3D(X2, Y2, Z2)

ax.set_aspect('auto')
ax.set_xlim(-2.0, 2.0)
ax.set_ylim(-2.0, 2.0)
ax.set_zlim(-2.0, 2.0)
plt.show()