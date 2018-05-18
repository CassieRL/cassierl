import numpy as np
import random


class CassieTrajectory:
    def __init__(self, filepath):
        n = 1 + 35 + 32 + 10 + 10 + 10
        data = np.fromfile(filepath, dtype=np.double).reshape((-1, n))
        self.time = data[:, 0]
        self.qpos = data[:, 1:36]
        self.qvel = data[:, 36:68]
        self.torque = data[:, 68:78]
        self.mpos = data[:, 78:88]
        self.mvel = data[:, 88:98]

    def state(self, t):
        tmax = self.time[-1]
        i = int((t % tmax) / tmax * len(self.time))
        return (self.qpos[i], self.qvel[i])

    def action(self, t):
        tmax = self.time[-1]
        i = int((t % tmax) / tmax * len(self.time))
        return (self.mpos[i], self.mvel[i], self.torque[i])

    def sample(self):
        i = random.randrange(len(self.time))
        return (self.time[i], self.qpos[i], self.qvel[i])


class CassieTrajectory2d(CassieTrajectory):
    def __init__(self, filepath):
        CassieTrajectory.__init__(self, filepath)

        """ remove 3D elements in qpos as follows """
        # left: abduction, yaw, knee_spring, heel_spring
        # right: same as left
        # self.qpos = np.delete(self.qpos, [7, 8, 11, 14,
        #                                   19, 20, 23, 26], axis=1)

        # convert base, left, and right's quaternions to Euler angles
        for i in range(self.qpos.shape[0]):
            # base
            z, y, x = self.quat2eul(self.qpos[i][3], self.qpos[i][4],
                                    self.qpos[i][5], self.qpos[i][6])
            self.qpos[i][3] = z
            self.qpos[i][4] = y
            self.qpos[i][5] = x
            # left
            z, y, x = self.quat2eul(self.qpos[i][15], self.qpos[i][16],
                                    self.qpos[i][17], self.qpos[i][18])
            self.qpos[i][15] = z
            self.qpos[i][16] = y
            self.qpos[i][17] = x
            # right
            z, y, z = self. quat2eul(self.qpos[i][27], self.qpos[i][28],
                                     self.qpos[i][29], self.qpos[i][30])
            self.qpos[i][27] = z
            self.qpos[i][28] = y
            self.qpos[i][29] = x
        self.qpos = np.delete(self.qpos, [1, 3, 5, 6, 7, 8, 11, 14, 15, 17, 18, 19, 20, 23, 26, 27, 29, 30, 31, 32, 33, 34], axis=1)

        """ remove 3D elements in qvel as follows """
        for i in range(self.qvel.shape[0]):
            # base
            z, y, x = self.quat2eul(self.qvel[i][3], self.qvel[i][4],
                                    self.qvel[i][5], self.qvel[i][6])
            self.qvel[i][3] = z
            self.qvel[i][4] = y
            self.qvel[i][5] = x
            # left
            z, y, x = self.quat2eul(self.qvel[i][15], self.qvel[i][16],
                                    self.qvel[i][17], self.qvel[i][18])
            self.qvel[i][15] = z
            self.qvel[i][16] = y
            self.qvel[i][17] = x
            # right
            z, y, z = self. quat2eul(self.qvel[i][27], self.qvel[i][28],
                                     self.qvel[i][29], self.qvel[i][30])
            self.qvel[i][27] = z
            self.qvel[i][28] = y
            self.qvel[i][29] = x
        self.qvel = np.delete(self.qvel, [1, 3, 5, 6, 7, 8, 11, 14, 15, 17, 18, 19, 20, 23, 26, 27, 29, 30, 31], axis=1)

        """ remove 3D elements in ctrl torques as follows """
        # left_abduction, left_yaw, right_abduction, right_yaw
        self.torque = np.delete(self.torque, [0, 1, 5, 6], axis=1)

    def quat2eul(self, w, x, y, z):
        """ Converts a quaternion into ZYX Euler angles. """
        t0 = 2.0*(w*x + y*z)
        t1 = 1.0 - 2.0*(x*x + y**2)
        X = np.arctan2(t0, t1)

        t2 = 2.0*(w*y - z*x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = np.arcsin(t2)

        t3 = 2.0*(w*z + x*y)
        t4 = 1.0 - 2.0*(y**2 + z*z)
        Z = np.arctan2(t3, t4)

        return Z, Y, X
