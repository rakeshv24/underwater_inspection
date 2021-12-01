import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import GenerateClouds

numPoints = 1000
# points = GenerateClouds.generateRandomCloud(50, 50, 50, 5, numPoints=10000)
points1 = GenerateClouds.generateUniformCloud(0, 0, 0, 2.5, numPoints=numPoints)
points2 = GenerateClouds.generateUniformCloud(0, 0, 0, 5, numPoints=numPoints)
points3 = GenerateClouds.generateUniformCloud(0, 0, 0, 7.5, numPoints=numPoints)

# genPoint1 = GenerateClouds.createTestPoint(5, math.pi/4, math.pi/4)
# genPoint2 = GenerateClouds.createTestPoint(7.5, math.pi/4, math.pi/4)

## Matplotlib Sample Code using 2D arrays via meshgrid
# point cloud
x1 = points1[:, 0]
y1 = points1[:, 1]
z1 = points1[:, 2]

x2 = points2[:, 0]
y2 = points2[:, 1]
z2 = points2[:, 2]

x3 = points3[:, 0]
y3 = points3[:, 1]
z3 = points3[:, 2]
# pitch = points[:, 3]
# yaw = points[:, 4]

# generated points for testing
# x1 = genPoint1[0]
# y1 = genPoint1[1]
# z1 = genPoint1[2]
#
# x2 = genPoint2[0]
# y2 = genPoint2[1]
# z2 = genPoint2[2]
#
# xQuiv, yQuiv, zQuiv = [0, x2], [0, y2], [0, z2]

fig = plt.figure()
ax = Axes3D(fig)
cloud1 = ax.scatter3D(x1, y1, z1, color='red')
cloud2 = ax.scatter3D(x2, y2, z2, color='blue')
cloud3 = ax.scatter3D(x3, y3, z3, color='green')

# for i in np.linspace(0, numPoints-1, num=10, dtype=int):
# 	quivPoint = GenerateClouds.createTestPoint(7.5, pitch[i], yaw[i])
# 	xQuiv, yQuiv, zQuiv = [0, quivPoint[0]], [0, quivPoint[1]], [0, quivPoint[2]]
#
# 	point1 = ax.scatter3D(x[i], y[i], z[i], color='red')
# 	point2 = ax.scatter3D(quivPoint[0], quivPoint[1], quivPoint[2], color='red')
# 	pointQuiv = ax.plot3D(xQuiv, yQuiv, zQuiv, color='red')

ax.set_aspect('auto')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.show()