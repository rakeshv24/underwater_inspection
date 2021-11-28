import numpy as np
import math

def generateRandomCloud(x, y, z, radius, numPoints=100):
	# function that generates random points on a sphere of input radius whose center is the input x,y,z coordinate
	# by default, the number of points we generate is 100
	# input: 	center of the sphere x, y, z
	#			radius of the sphere
	#			number of points to generate (default 100)
	# outputs:	ndarray -> row contains a random position x, y, z and a heading (pitch, yaw)

	# creating an array of the correct size
	points = np.zeros((numPoints, 5), dtype=float)

	# epsilon to add to our upper bound since random.uniform doesn't include the upper bound
	epsilon = 0.000001

	# generating random pitch values to calculate new coordinates
	lowerPitch = -1 * (math.pi / 2)
	upperPitch = (math.pi / 2) + epsilon
	randCoorPitchValues = np.random.uniform(lowerPitch, upperPitch, numPoints)

	# generating random yaw values
	lowerYaw = 0
	upperYaw = 2 * math.pi
	randCoorYawValues = np.random.uniform(lowerYaw, upperYaw, numPoints)

	# calculating random x, y, z positions
	xCoors = radius * np.cos(randCoorPitchValues) * np.sin(randCoorYawValues)
	yCoors = radius * np.sin(randCoorPitchValues) * np.sin(randCoorYawValues)
	zCoors = radius * np.cos(randCoorYawValues)

	# loading new coordinates into the matrix and adding the origin offsets
	points[:, 0] = xCoors + x
	points[:, 1] = yCoors + y
	points[:, 2] = zCoors + z

	# generating random heading angles
	# generating random pitch values
	lowerPitch = -1 * (math.pi / 4)
	upperPitch = (math.pi / 4) + epsilon
	randPitchValues = np.random.uniform(lowerPitch, upperPitch, numPoints)

	# generating random yaw values
	lowerYaw = 0
	upperYaw = 2 * math.pi
	randYawValues = np.random.uniform(lowerYaw, upperYaw, numPoints)

	# loading headings into the matrix
	points[:, 3] = randPitchValues
	points[:, 4] = randYawValues

	return points

def generateUniformCloud(x, y, z, radius, numPoints=100):
	# function that generates uniform points around the sphere of input radius whose center is the input x,y,z coordinate
	# by default, the number of points we generate is 100. all generated headings are orthogonal to the sphere
	# input: 	center of the sphere x, y, z
	#			radius of the sphere
	#			number of points to generate (default 100)
	# outputs:	ndarray -> row contains a random position x, y, z and a heading (pitch, yaw)

	# creating an array of the correct size
	points = np.zeros((numPoints, 5), dtype=float)

	i = np.arange(0, numPoints, dtype=float) + 0.5
	goldenRatio = (1 + 5 ** 0.5) / 2
	coorYaw = np.arccos(1 - 2 * i / numPoints)
	coorPitch = 2 * math.pi * i / goldenRatio

	# calculating random x, y, z positions
	xCoors = radius * np.cos(coorPitch) * np.sin(coorYaw)
	yCoors = radius * np.sin(coorPitch) * np.sin(coorYaw)
	zCoors = radius * np.cos(coorYaw)

	# loading calculated values into matrix
	# loading the x, y, z points with the origin offsets
	points[:, 0] = xCoors + x
	points[:, 1] = yCoors + y
	points[:, 2] = zCoors + z
	# pitch and yaw are unaffected by the origin offsets
	# heading tangent to the sphere
	# points[:, 3] = -np.arctan(yCoors, np.sqrt(xCoors**2 + zCoors**2))
	# points[:, 4] = np.arctan(zCoors, xCoors) - math.pi/2
	# maybe normal to the sphere???
	V = np.array([xCoors, yCoors, zCoors])
	normV = np.apply_along_axis(np.linalg.norm, 1, V)

	VP = np.full(V.shape, 0)
	VP[:, :2] = V[:, :2]
	normVP = np.apply_along_axis(np.linalg.norm, 1, VP)

	VPP = np.full(V.shape, 0)
	VPP[:, 1] = V[:, 1]
	normVPP = np.apply_along_axis(np.linalg.norm, 1, VPP)

	# pitch and yaw
	points[:, 3] = np.arccos(np.sum(VP * VPP, axis=1)/(normVP * normVPP))
	points[:, 4] = np.arccos(np.sum(V * VP, axis=1)/(normV * normVP))

	return points

