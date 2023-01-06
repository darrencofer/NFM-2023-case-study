'''
Neural Network for Mid-Air Collision Avoidance DARPA Assured Autonomy
'''

import sys
import numpy as np

## %
# Path to Marabou folder if you did not export it

# sys.path.append('/home/USER/git/Marabou')
from maraboupy import Marabou
from maraboupy import MarabouCore

# %%
# Path to NNet file
nnetFile = "../../../2021-09-06-SAC-VECTOR-LR-2.nnet"

# %%
# Load the network from NNet file, and set a lower bound on first output variable
net1 = Marabou.read_nnet(nnetFile)

inputVars = net1.inputVars[0][0]
outputVars = net1.outputVars[0][0]

# property 
net1.setLowerBound(outputVars[0], 15)
#net1.setUpperBound(outputVars[0], 1.0)

# input range
net1.setLowerBound(inputVars[0], 0)
net1.setUpperBound(inputVars[0], 1)
net1.setLowerBound(inputVars[1], 0)
net1.setUpperBound(inputVars[1], 1)
net1.setLowerBound(inputVars[2], 0)
net1.setUpperBound(inputVars[2], 1)
net1.setLowerBound(inputVars[3], -0.3)
net1.setUpperBound(inputVars[3], 0.4)
net1.setLowerBound(inputVars[4], -1.)
net1.setUpperBound(inputVars[4], 1.)
net1.setLowerBound(inputVars[5], -1.)
net1.setUpperBound(inputVars[5], 1.)
net1.setLowerBound(inputVars[7], -1.)
net1.setUpperBound(inputVars[7], 1.)

# input constraint angle_to_goal = 30 degrees
net1.setLowerBound(inputVars[6], 0.16667)
net1.setUpperBound(inputVars[6], 0.16667)

coeffs = [0, -1, 0.5, 0, 0, 0, 0, 0] 
net1.addEquality(inputVars, coeffs, 0)

# options
options = Marabou.createOptions(timeoutInSeconds=30)

# %%
# Solve Marabou query
exitCode, vals1, stats1 = net1.solve(options=options)

for i in range(0, 8):
    print(vals1[i])


# %%
# Example statistics
#stats1.getUnsignedAttribute(StatisticsUnsignedAttribute.NUM_SPLITS)
#stats1.getTotalTimeInMicro()


