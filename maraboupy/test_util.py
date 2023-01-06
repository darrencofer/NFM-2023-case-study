'''
Neural Network for Mid-Air Collision Avoidance DARPA Assured Autonomy
'''

import sys
import numpy as np
from myenv.utils import clamp, unwind_angle, scale, heading_from_coordinates

## %
# Path to Marabou folder if you did not export it

# sys.path.append('/home/USER/git/Marabou')
from maraboupy import Marabou
from maraboupy import MarabouCore



position = [30, 20]
self_position = np.array(position, dtype=float)  # m, x-right, y-up
position = [10, 20]
xy_position = np.array(position, dtype=float)  # m, x-right, y-up 
bearing = heading_from_coordinates(self_position, xy_position)

print(bearing)
