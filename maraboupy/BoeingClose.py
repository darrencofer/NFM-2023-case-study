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
net1.setLowerBound(outputVars[0], 1.0)
net1.setUpperBound(outputVars[0], 1.0)

# input constraint 
net1.setLowerBound(inputVars[1], 0.)
net1.setUpperBound(inputVars[1], 0.)
net1.setLowerBound(inputVars[5], 0.)
net1.setUpperBound(inputVars[5], 0.)
net1.setLowerBound(inputVars[6], 0.)
net1.setUpperBound(inputVars[6], 0.)
net1.setLowerBound(inputVars[7], 0.)
net1.setUpperBound(inputVars[7], 0.)

# %%
# Solve Marabou query
exitCode, vals1, stats1 = net1.solve()

