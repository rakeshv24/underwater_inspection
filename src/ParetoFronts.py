import numpy as np
from scipy.spatial import distance_matrix

def normalize(mat, axis=0):
	return (mat - np.min(mat, axis=axis)) / np.ptp(mat, axis=axis)

def calcSingleSPStrength(objectiveValues, i):
	strengths = np.logical_and((objectiveValues[i, 0] > objectiveValues[:, 0]),
							   (objectiveValues[i, 1] > objectiveValues[:, 1]),
							   (objectiveValues[i, 2] > objectiveValues[:, 2])).sum()

	return strengths

def calcSPRawFitness(objectiveValues, strengths, i):
	# finding which points dominate the current point i
	indxs = np.where(np.logical_and((objectiveValues[i, 0] < objectiveValues[:, 0]),
								   	(objectiveValues[i, 1] < objectiveValues[:, 1]),
								   	(objectiveValues[i, 2] < objectiveValues[:, 2])))[0]

	# returning the sum of the strengths of each of those indexes
	return strengths[indxs].sum()

def calcSingleSPDensity(distMat, k, i):
	# extracting the current distances
	dists = np.sort(distMat[i, :])
	# getting the kth dist
	dist = dists[0, k]
	# calculating the density for the current point
	density = 1 / (dist + 2)

	return density

def calcSPDensityValues(objectiveValues, k):
	# calculating all distances
	normMat = normalize(objectiveValues)
	distMat = distance_matrix(normMat, normMat)

	indxRange = np.arange(0, len(objectiveValues)).reshape((-1, 1))
	densityValues = np.apply_along_axis(lambda i: calcSingleSPDensity(distMat, k, i), 1, indxRange)

	return densityValues

def calcSPFitnessValues(objectiveValues):
	# function to calculate the fitness values using strength pareto fronts
	# input:	Objective values (corresponding battery, path length, map percentage for each x,y,z point on the
	# 			spherical cap)population of points (x, y, z points of the spherical cap)
	# outputs:	fitness values of all the points in corresponding order

	# creating a range to apply along for strength, fitness, and density calculations
	indxRange = np.arange(0, len(objectiveValues)).reshape((-1, 1))

	# calculating the strengths by counting how many points are less than our current point
	strengths = np.apply_along_axis(lambda i: calcSingleSPStrength(objectiveValues, i), 1, indxRange)

	# calculating the raw fitness values by adding the strengths of points that dominate the current point
	rawFitnessValues = np.apply_along_axis(lambda i: calcSPRawFitness(objectiveValues, strengths, i), 1, indxRange)

	# calculating density
	k = round(np.sqrt(len(objectiveValues)))
	densityValues = calcSPDensityValues(objectiveValues, k)

	return rawFitnessValues + densityValues