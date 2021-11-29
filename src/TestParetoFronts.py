import numpy as np

import ParetoFronts

piq = [100, 100, 100]
other1 = [1, 2, 3]
other2 = [4, 5, 6]
other3 = [7, 8, 9]

objectiveValues = np.array([piq, other1, other2, other3])

fitnessValues = ParetoFronts.calcSPFitnessValues(objectiveValues)

print(fitnessValues)