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

def getRotationMatrix(pitch, yaw):
	# function to calculate a partial rotation matrix using the pitch and the yaw
	# inputs: 	pitch of the auv
	#			yaw of the auv
	# outputs: 	rotation matrix

	# calculating yaw matrix
	yawMat = np.array([[np.cos(yaw), -np.sin(yaw), 0],
					   [np.sin(yaw), np.cos(yaw), 0],
					   [0, 0, 1]])


	# calculating pitch matrix
	pitchMat = np.array([[np.cos(pitch), 0, -np.sin(pitch)],
						 [0, 1, 0],
						 [np.sin(pitch), 0, np.cos(pitch)]])

	# calculating partial rotation mat
	return np.dot(yawMat, pitchMat)

def calcHeading(sphereRadius, x, y, z):
	pitch = np.arcsin(z / sphereRadius)
	yaw = np.arctan(y / x)

	return pitch, yaw

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

	# calculating the pitch and yaw values that yield normal headings
	# originVecs = np.zeros((numPoints, 3), dtype=float)
	# radiusVecs = np.full((numPoints, 1), radius, dtype=float).T
	# originVecs[:, 0] = radiusVecs
	#
	# pitchVecs = np.zeros((numPoints, 3), dtype=float)
	# pitchVecs[:, 0] = xCoors
	# pitchVecs[:, 2] = zCoors
	#
	# yawVecs = np.zeros((numPoints, 3), dtype=float)
	# yawVecs[:, 0] = xCoors
	# yawVecs[:, 1] = yCoors
	#
	# points[:, 3] = calcInnerAngle(originVecs, pitchVecs)
	# points[:, 4] = calcInnerAngle(originVecs, yawVecs)

	pitch, yaw = calcHeading(radius, xCoors, yCoors, zCoors)

	points[:, 3] = pitch
	points[:, 4] = yaw

	return points

def createTestPoint(sphereRadius, pitch, yaw):
	# initial point
	origin = np.array([sphereRadius, 0, 0])
	# getting rotation matrix
	rotMat = getRotationMatrix(pitch, yaw)
	# rotate origin to the current heading
	rotatedOrigin = rotMat.dot(origin.T).T

	return rotatedOrigin