import numpy as np

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

def sliceSphericalCap(points, x, y, z, sphereRadius, pitch, yaw, capRadius):
	# function to get the points that make up a spherical cap
	# inputs: 	full spherical cloud
	#			x, y, z point that represents the origin
	# 			pitch of the auv
	#			yaw of the auv
	#			cap radius
	# outputs: 	subset of the points input that make up the cap

	# calculating x, y, z point for current heading
	# start at the system origin along the x axis
	origin = np.array([sphereRadius, 0, 0])
	# getting rotation matrix
	rotMat = getRotationMatrix(pitch, yaw)
	# rotate origin to the current heading
	rotatedOrigin = rotMat.dot(origin.T).T

	# calculating approx for comparison radius
	compDist = np.sqrt(capRadius**2 + (sphereRadius - np.sqrt(sphereRadius**2 - capRadius**2))**2)

	# translating the rotated origin to the sphere's frame
	translatedOrigin = rotatedOrigin + [x, y, z]

	# finding the points in the original cloud that make up the desired cap
	# getting a subset of the x, y, z points from the point cloud
	xyzPoints = points[:, :3]
	# calculating the distance from the heading point (translatedOrigin) to each point on the sphere
	dists = np.apply_along_axis(lambda lArg: np.linalg.norm(lArg - translatedOrigin), 1, xyzPoints)
	# finding the indices where the distance is less than or equal to the cap radius
	capPointIndxs = np.where(dists <= compDist)

	# returning the points where we are within the cone.
	return points[capPointIndxs]

def slicePatch(points, pitch ,yaw, sphereRadius, patchHeight, patchWidth):
	# function to get the points that make up a patch
	# inputs: 	full spherical cloud
	# 			pitch of the auv
	#			yaw of the auv
	#			patch height
	# 			patch width
	# outputs: 	subset of the points that make up the patch

	# calculating phi angles
	phiWidth = np.arcsin((patchWidth / 2) / sphereRadius)
	phiHeight = np.arcsin((patchHeight / 2) / sphereRadius)

	# calculating the 4 corners of the patch
	topRight 	= [sphereRadius * np.cos(-phiWidth), sphereRadius * np.sin(-phiWidth), sphereRadius * np.sin(phiHeight)]
	topLeft		= [sphereRadius * np.cos(phiWidth), sphereRadius * np.sin(phiWidth), sphereRadius * np.sin(phiHeight)]
	botRight 	= [sphereRadius * np.cos(-phiWidth), sphereRadius * np.sin(-phiWidth), sphereRadius * np.sin(-phiHeight)]
	botLeft		= [sphereRadius * np.cos(phiWidth), sphereRadius * np.sin(phiWidth), sphereRadius * np.sin(-phiHeight)]

	# loading corners into an array for translation
	originCorners = np.array([topRight, topLeft, botRight, botLeft])

	# getting rotation matrix and translating points
	rotMat = getRotationMatrix(pitch, yaw)
	translatedCorners = rotMat.dot(originCorners.T).T

	# finding bounds for the rectangle
	# determining x bounds
	xLB = translatedCorners[:, 0].min()
	xUB = translatedCorners[:, 0].max()

	# determining y bounds
	yLB = translatedCorners[:, 1].min()
	yUB = translatedCorners[:, 1].max()

	# determining z bounds
	zLB = translatedCorners[:, 2].min()
	zUB = translatedCorners[:, 2].max()

	# selecting points based on our bounds
	# x points
	xPoints = points[:, 0]
	if (xUB == xLB):
		# if the bounds are the same, check the sign for which direction they are in
		if xUB < 0:
			xPointIndxs = np.where(xPoints <= xUB)[0]
		else:
			xPointIndxs = np.where(xPoints >= xUB)[0]
	else:
		# the bounds aren't the same so we compile the points in between the bounds
		xPointLBIndxs = np.where(xPoints >= xLB)[0]
		xPointUBIndxs = np.where(xPoints <= xUB)[0]
		xPointIndxs = np.intersect1d(xPointLBIndxs, xPointUBIndxs)

	# y points
	yPoints = points[:, 1]
	if (yUB == yLB):
		# if the bounds are the same, check the sign for which direction they are in
		if yUB < 0:
			yPointIndxs = np.where(yPoints <= yUB)[0]
		else:
			yPointIndxs = np.where(yPoints >= yUB)[0]
	else:
		# the bounds aren't the same so we compile the points in between the bounds
		yPointLBIndxs = np.where(yPoints >= yLB)[0]
		yPointUBIndxs = np.where(yPoints <= yUB)[0]
		yPointIndxs = np.intersect1d(yPointLBIndxs, yPointUBIndxs)

	# z points
	zPoints = points[:, 2]
	if (zUB == yLB):
		# if the bounds are the same, check the sign for which direction they are in
		if zUB < 0:
			zPointIndxs = np.where(zPoints <= zUB)[0]
		else:
			zPointIndxs = np.where(zPoints >= zUB)[0]
	else:
		# the bounds aren't the same so we compile the points in between the bounds
		zPointLBIndxs = np.where(zPoints >= zLB)[0]
		zPointUBIndxs = np.where(zPoints <= zUB)[0]
		zPointIndxs = np.intersect1d(zPointLBIndxs, zPointUBIndxs)

	xyPointIndxs = np.intersect1d(xPointIndxs, yPointIndxs)
	xyzPointIndxs = np.intersect1d(xyPointIndxs, zPointIndxs)

	return points[xyzPointIndxs], translatedCorners