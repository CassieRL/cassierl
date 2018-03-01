"""
Makes Cassie squat up and down. This is purely physics. There is no RL involved.
"""

import cassie2d
import numpy as np

env = cassie2d.Cassie2dEnv()
env.reset()

freq = 0.5
w = freq*3.1415
t = 0.0

while (True):
    env.standing_controller_jacobian(0.7 + 0.25*np.sin(w*t), 0.25*np.cos(w*t))
    t = t + 0.0005
