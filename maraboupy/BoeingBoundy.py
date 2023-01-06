'''
Neural Network for Mid-Air Collision Avoidance DARPA Assured Autonomy
'''

import sys
import numpy as np
import math
from copy import deepcopy

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
net1.setLowerBound(outputVars[0], 2)
#net1.setLowerBound(outputVars[0], 0.01)
#net1.setLowerBound(outputVars[0], 0)
#net1.setUpperBound(outputVars[0], -2)

# input range
# dist_to_intruder
net1.setLowerBound(inputVars[0], 0.2001)
net1.setUpperBound(inputVars[0], 0.2001)
# dist_to_vector
net1.setLowerBound(inputVars[1], 0)
net1.setUpperBound(inputVars[1], 1)
# dist_to_init
#net1.setLowerBound(inputVars[2], 0)
net1.setLowerBound(inputVars[2], 0.1)
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
# rel_heading
net1.setLowerBound(inputVars[5], -2/180)
net1.setUpperBound(inputVars[5], -2/180)

# angle_to_intruder
net1.setLowerBound(inputVars[4], 3/180.)
net1.setUpperBound(inputVars[4], 177/180.)

# rel_speed
net1.setLowerBound(inputVars[3], 0)
net1.setUpperBound(inputVars[3], 0)

# vars * coeffs <= scalar
# 180 >  intruder_angle - relative_heading > 0
#coeffs = [0, 0, 0, 0, 1, -1, 0, 0] 
#net1.addInequality(inputVars, coeffs, 1)
#coeffs = [0, 0, 0, 0, -1, 1, 0, 0] 
#net1.addInequality(inputVars, coeffs, 0)


# 180 > intruder_angle - turn_rate* ∆t > 0
#net1.setUpperBound(inputVars[4], 0.5)
#net1.setLowerBound(inputVars[4], 0.95)

# intruder_angle - turn_rate* ∆t < 90
#k = 3/180
#vars = [inputVars[4], outputVars[0]]
#coeffs = [-1, k] 
#net1.addInequality(vars, coeffs, 0.5)

# options
options = Marabou.createOptions(timeoutInSeconds=30)

# %%
f = open("results.txt", "w")
for i in range(5, 180, 5):
  coeffs = [1] 
  vars = [inputVars[4]]
  net2 = deepcopy(net1)
  net1.addEquality(vars, coeffs, i/180)
  # Solve Marabou query
  exitCode, vals1, stats1 = net1.solve(options=options)
  net1 = deepcopy(net2)

  f.write(str(i) + '\t')
  f.write(str(len(vals1)) + '\n')
  if (len(vals1) > 0):
   inputs = np.random.rand(8)
   for i in range(0, 8):
      inputs[i] = vals1[i]
      f.write('inputs[' + str(i) + ']= ' + str(vals1[i]) + '\n')
   #print('---')
   #inputs = vals1[0:7]
   outputs = net1.evaluateWithMarabou(inputs)
   f.write('output = ' + str(outputs[0][0]) + '\n')

   # intruder_angle -turn_rate ∆t
   intruder_angle = vals1[4]*math.pi 
   rel_heading = vals1[5]*math.pi
   rel_speed = vals1[3]
   turn_rate = math.tanh(outputs[0][0])*3/180*math.pi
   f.write('turn_rate = ' + str(math.tanh(outputs[0][0])*3) + '\n')

   phi = intruder_angle - turn_rate
   alpha = intruder_angle - rel_heading
   derivative = (1 + rel_speed)*math.cos(alpha) - math.cos(phi)
   f.write('derivative = ' + str(derivative) + '\n')
    
f.close()
