import numpy as np

import ParetoFronts

# for testing, we look at a point of interest and some other points
# x -> battery
# y -> path length
# z -> map coverage
poi = [1, 2, 2]
other1 = [1, 2, 3]
other2 = [4, 5, 6]
other3 = [7, 8, 9]

objectiveValues = np.array([poi, other1, other2, other3])

fitnessValues = ParetoFronts.calcSPFitnessValues(objectiveValues)

print(fitnessValues)