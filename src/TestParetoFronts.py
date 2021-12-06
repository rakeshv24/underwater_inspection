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

test = [[98.0, -2.0, 0.8338907361030579], [98.0, -2.0, 0.9951386451721191], [98.0, -2.0, 0.8144116997718811], [98.0, -2.0, 0.8975814580917358], [98.0, -2.0, 0.9984893202781677], [98.0, -2.0, 0.8913610577583313]]

objectiveValues = np.array(test)

fitnessValues = ParetoFronts.calcSPFitnessValues(objectiveValues)

print(fitnessValues)