import numpy as np

from cassie2d_trajectory import Cassie2dEnv
from cassie2d_trajectory import Cassie2dTraj

traj = Cassie2dTraj('../trajectory/stepdata.bin')

env = Cassie2dEnv()
env.reset()
t = 0.0

with open('trajectory.csv', 'w') as f:
    f.write('time,')
    f.write('left_hip_angle,left_knee_angle,left_toe_angle,')
    f.write('right_hip_angle,right_knee_angle,right_toe_angle,')
    f.write('left_hip_qvel,left_knee_qvel,left_toe_qvel,')
    f.write('right_hip_qvel,right_knee_qvel,right_toe_qvel\n')
    # f.write(str(traj.qpos[0][0]) + ', ' + str(traj.qpos[0][1]) + ', ' + str(traj.qpos[0][2]) + ',\n')
    # f.write(str(traj.qvel[0][0]) + ', ' + str(traj.qvel[0][1]) + ', ' + str(traj.qvel[0][2]) + ',\n')
    # f.write(str(traj.qpos[0][3]) + ', ' + str(traj.qpos[0][4]) + ', ' + str(traj.qpos[0][5]) + ', ' + str(traj.qpos[0][6]) + ', ' + str(traj.qpos[0][7]) + ',\n')
    # f.write(str(traj.qvel[0][3]) + ', ' + str(traj.qvel[0][4]) + ', ' + str(traj.qvel[0][5]) + ', ' + str(traj.qvel[0][6]) + ', ' + str(traj.qvel[0][7]) + ',\n')
    # f.write(str(traj.qpos[0][8]) + ', ' + str(traj.qpos[0][9]) + ', ' + str(traj.qpos[0][10]) + ', ' + str(traj.qpos[0][11]) + ', ' + str(traj.qpos[0][12]) + '\n')
    # f.write(str(traj.qvel[0][8]) + ', ' + str(traj.qvel[0][9]) + ', ' + str(traj.qvel[0][10]) + ', ' + str(traj.qvel[0][11]) + ', ' + str(traj.qvel[0][12]) + '\n')

while (True):
    env.step(t)
    t += 0.0005
