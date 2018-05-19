import numpy as np

from cassie2d_trajectory import Cassie2dEnv

env = Cassie2dEnv()
env.reset()
t = 0.0

with open('trajectory.csv', 'w') as f:
    f.write('time,')
    f.write('left_hip_angle,left_knee_spring_angle,left_toe_angle,')
    f.write('right_hip_angle,right_knee_spring_angle,right_toe_angle,')
    f.write('left_hip_qvel,left_knee_spring_qvel,left_toe_qvel,')
    f.write('right_hip_qvel,right_knee_spring_qvel,right_toe_qvel\n')

while (True):
    for _ in range(10000):
        pass
    env.step(t)
    t += 0.0005
