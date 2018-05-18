import numpy as np
import loadstep


traj = loadstep.CassieTrajectory('stepdata.bin')
traj2d = loadstep.CassieTrajectory2d('stepdata.bin')

# with open('test.txt', 'w') as f:
#     f.write('--------------Before--------------')
#     f.write('traj.qpos[0] = ')
#     f.write(str(traj.qpos[0]))

#     f.write('--------------After--------------')
#     f.write('traj2d.qpos[0] = ')
#     f.write(str(traj2d.qpos[0]))

#     for i in range(20):
#         f.write('--------------body----Row %d----------\n' % i)
#         qw = traj2d.qpos[i][3]
#         qx = traj2d.qpos[i][4]
#         qy = traj2d.qpos[i][5]
#         qz = traj2d.qpos[i][6]
#         f.write('Quat (wxyz): quat2eul([' + str(traj.qpos[i][3]) + ' '
#                                           + str(traj.qpos[i][4]) + ' '
#                                           + str(traj.qpos[i][5]) + ' '
#                                           + str(traj.qpos[i][6]) + '])\n')
#         f.write('Euler (zyx): ' + str(tuple(traj2d.qpos[i][3:6])) + '\n')
#         f.write('traj.qpos[i][9] = ' + str(traj.qpos[i][9]) + '\n')
#         f.write('traj2d.qpos[i][6] = ' + str(traj2d.qpos[i][6]) + '\n')

print(traj2d.qpos.shape)
print(traj2d.qvel.shape)

print()
print(traj2d.qpos[0])
print(traj2d.qvel[0])
