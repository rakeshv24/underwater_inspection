import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

import GenerateClouds

# points = GenerateClouds.generateRandomCloud(50, 50, 50, 5, numPoints=10000)
points = GenerateClouds.generateUniformCloud(50, 50, 50, 5, numPoints=1000)
genPoint = GenerateClouds.createTestPoint(5, math.pi/4, math.pi/4)

## Matplotlib Sample Code using 2D arrays via meshgrid
X = points[:, 0]
Y = points[:, 1]
Z = points[:, 2]

X2 = genPoint[0]
Y2 = genPoint[1]
Z2 = genPoint[2]

fig = plt.figure()
ax = Axes3D(fig)
surf = ax.scatter3D(X, Y, Z)
point = ax.scatter3D(X2, Y2, Z2)

ax.set_aspect('auto')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.show()