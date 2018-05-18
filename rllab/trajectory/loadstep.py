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
        # left: abduction, yaw, knee_spring, heel_spring, quaternion(conrod)
        # right: same as left
        self.qpos = np.delete(self.qpos, [7, 8, 11, 14, 15, 16, 17, 18,
                                          19, 20, 23, 26, 27, 28, 29, 30], axis=1)
        # convert body's quaternions to Euler angles
        for i in range(self.qpos.shape[0]):
            z, y, x = self.quat2eul(self.qpos[i][3], self.qpos[i][4],
                                    self.qpos[i][5], self.qpos[i][6])
            self.qpos[i][3] = z
            self.qpos[i][4] = y
            self.qpos[i][5] = x
        self.qpos = np.delete(self.qpos, [6], axis=1)

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
