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
# Load the network from NNet file, and set a bound on first output variable
net1 = Marabou.read_nnet(nnetFile)

inputVars = net1.inputVars[0][0]
outputVars = net1.outputVars[0][0]

# property 
net1.setLowerBound(outputVars[0], 0)
net1.setUpperBound(outputVars[0], 5)

# input range
# dist_to_intruder
net1.setLowerBound(inputVars[0], 0.099)
net1.setUpperBound(inputVars[0], 0.099)
# dist_to_vector
net1.setLowerBound(inputVars[1], 0)
net1.setUpperBound(inputVars[1], 1)
# dist_to_init
#net1.setLowerBound(inputVars[2], 0)
net1.setLowerBound(inputVars[2], 0.001)
net1.setUpperBound(inputVars[2], 1)
# rel_speed
net1.setLowerBound(inputVars[3], -0.3)
net1.setUpperBound(inputVars[3], 0.4)
# angle_to_intruder
net1.setLowerBound(inputVars[4], -1.)
net1.setUpperBound(inputVars[4], 1.)
# rel_heading
net1.setLowerBound(inputVars[5], -1.)
net1.setUpperBound(inputVars[5], 1.)
# angle_to_vector
net1.setLowerBound(inputVars[7], -1.)
net1.setUpperBound(inputVars[7], 1.)

# input constraint dist_to_init * sin(angle_to_goal) = dist_to_vector
# set angle_to_goal = 30 degrees
angle_to_goal = 30/180
net1.setLowerBound(inputVars[6], angle_to_goal)
net1.setUpperBound(inputVars[6], angle_to_goal)

coeffs = [0, -1, 0.5, 0, 0, 0, 0, 0] 
net1.addEquality(inputVars, coeffs, 0)

# conditions for crossing safe/unsafe dist 
# vars * coeffs <= scalar
# 270 >  intruder_angle - relative_heading > 90
coeffs = [0, 0, 0, 0, 1, -1, 0, 0] 
net1.addInequality(inputVars, coeffs, 1.5)
coeffs = [0, 0, 0, 0, -1, 1, 0, 0] 
net1.addInequality(inputVars, coeffs, -0.5)

# -90 < intruder_angle - turn_rate* ∆t
net1.setUpperBound(inputVars[4], 0.5)
#net1.setLowerBound(inputVars[4], 0.95)

# intruder_angle - turn_rate* ∆t < 90
k = 3/180
vars = [inputVars[4], outputVars[0]]
coeffs = [-1, k] 
net1.addInequality(vars, coeffs, 0.5)

# options
options = Marabou.createOptions(timeoutInSeconds=30)

# %%
for i in range(-180, 180):
  coeffs = [1] 
  vars = [inputVars[4]]
  net1.addEquality(vars, coeffs, i/180)
  # Solve Marabou query
  exitCode, vals1, stats1 = net1.solve(options=options)

inputs = np.random.rand(8)
#print(len(vals1))
for i in range(0, 8):
    inputs[i] = vals1[i]
    print(vals1[i])
print('---')
#inputs = vals1[0:7]
outputs = net1.evaluateWithMarabou(inputs)
print('output = ', outputs[0][0])

# intruder_angle -turn_rate ∆t
intruder_angle = vals1[4]*180 
rel_heading = vals1[5]*180
rel_speed = vals1[3]
turn_rate = outputs[0][0]

phi = intruder_angle - turn_rate
theta = 180 - intruder_angle + rel_heading
neg_derivative = math.cos(phi) + (1 + rel_speed)*math.cos(theta)
print('neg_derivative = ', neg_derivative)
