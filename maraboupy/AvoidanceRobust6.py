'''
Neural Network for Mid-Air Collision Avoidance DARPA Assured Autonomy
'''

import sys
import numpy as np
import math

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
#net1.setLowerBound(outputVars[0], 0)
net1.setUpperBound(outputVars[0], 0)

inputs = [
 0.101,
0.0775429194821185,
 0.155085838964237,
0.0,
0.4444444444444444,
 -0.011111111111111112,
 0.16666666666666666,
0.45367993014418134
	]

angle = 180 * inputs[6]
coeff = math.sin(math.radians(angle))
inputs[1] = inputs[2] * coeff

delta = 0.05

lb = []
ub = []
lb = [0 for i in range(0, 8)] 
ub = [0 for i in range(0, 8)] 
for i in range(0, 8):
    lb[i] = inputs[i] - delta
for i in range(0, 8):
    ub[i] = inputs[i] + delta

# input constraint 
for i in range(0, 8):
    net1.setLowerBound(inputVars[i], lb[i])
    net1.setUpperBound(inputVars[i], ub[i])

net1.setLowerBound(inputVars[6], inputs[6])
net1.setUpperBound(inputVars[6], inputs[6])

# fix speed
net1.setLowerBound(inputVars[3], inputs[3])
net1.setUpperBound(inputVars[3], inputs[3])

# fix relative heading
net1.setLowerBound(inputVars[5], inputs[5])
net1.setUpperBound(inputVars[5], inputs[5])

# input constraint angle_to_goal = 30 degrees
# 30/180 = 0.16667, sin30 = 0.5
coeffs = [0, -1, coeff, 0, 0, 0, 0, 0]
net1.addEquality(inputVars, coeffs, 0)

# %%
# Solve Marabou query
exitCode, vals1, stats1 = net1.solve()

#for i in range(0, 8):
#    print(vals1[i])

# %%
# Example statistics
#stats1.getUnsignedAttribute(StatisticsUnsignedAttribute.NUM_SPLITS)
#stats1.getTotalTimeInMicro()
