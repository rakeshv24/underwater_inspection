import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import GenerateClouds

points = GenerateClouds.generateUniformCloud(0, 0, 0, 2, numPoints=1000)

## Matplotlib Sample Code using 2D arrays via meshgrid
X = points[:, 0]
Y = points[:, 1]
Z = points[:, 2]

fig = plt.figure()
ax = Axes3D(fig)
surf = ax.scatter3D(X, Y, Z)

ax.set_aspect('auto')
plt.show()