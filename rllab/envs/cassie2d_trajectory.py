import numpy as np
import random


class Cassie3dTraj:
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


class Cassie2dTraj(Cassie3dTraj):
    def __init__(self, filepath):
        Cassie3dTraj.__init__(self, filepath)

        """ Converting 3D qpos to 2D qpos
        The ordering of 3D qpos is as follows:

                0  1  2            3 4 5 6
        BASE    x  y  z  base_quat(w x y z)

                7          8    9    10    11           12     13   14          15           16                       17 18 19 20
        LEFT    abduction  yaw  hip  knee  knee_spring  ankle  toe  anke_rod_1  ankle_rod_2  heel_spring  conrod_quat(w  x  y  z)

                21         22   23   24    25           26     27   28          29           30                       31 32 33 34
        RIGHT   abduction  yaw  hip  knee  knee_spring  ankle  toe  anke_rod_1  ankle_rod_2  heel_spring  conrod_quat(w  x  y  z)

        Convert Quaternions to Euler angles but only keep the Y-axis rotation
        Remove abduction, yaw, knee_spring, ankle_rod_1, ankle_rod_2, heel_spring of two legs
        2D qpos should look like like StateGeneral() in cassie2d_structs.py, but ignore the velocity fields

        The resulting 2D qpos should be as follows:

                0  1  2
        BASE    x  z  base_euler_Y

                3    4     5      6    7
        LEFT    hip  knee  ankle  toe  rod_euler_Y

                8    9     10     11   12
        RIGHT   hip  knee  ankle  toe  rod_euler_Y
        """
        # convert base, left, and right's quaternions to Euler angles
        # but only keep the Y angle since we are removing the y-axis
        for i in range(self.qpos.shape[0]):
            # base
            _, self.qpos[i][4], _ = self.quat2eul(self.qpos[i][3],
                                                  self.qpos[i][4],
                                                  self.qpos[i][5],
                                                  self.qpos[i][6])
            # left
            _, self.qpos[i][18], _ = self.quat2eul(self.qpos[i][17],
                                                   self.qpos[i][18],
                                                   self.qpos[i][19],
                                                   self.qpos[i][20])
            # right
            _, self.qpos[i][32], _ = self. quat2eul(self.qpos[i][31],
                                                    self.qpos[i][32],
                                                    self.qpos[i][33],
                                                    self.qpos[i][34])
        # remove 3D components
        self.qpos = np.delete(self.qpos, [1, 3, 5, 6,
                                          7, 8, 11, 14, 15, 16, 17, 19, 20,
                                          21, 22, 25, 28, 29, 30, 31, 33, 34],
                              axis=1)

        """ Converting 3D qvel to 2D qvel
        The ordering of 3D qvel is as follows:

                0  1  2                  3 4 5
        BASE    x  y  z  base_quat_deriv(x y z)

                6          7    8    9     10           11     12   13          14           15                             16 17 18
        LEFT    abduction  yaw  hip  knee  knee_spring  ankle  toe  anke_rod_1  ankle_rod_2  heel_spring  conrod_quat_deriv(x  y  z)

                19         20   21   22    23           24     25   26          27           28                             29 30 31
        RIGHT   abduction  yaw  hip  knee  knee_spring  ankle  toe  anke_rod_1  ankle_rod_2  heel_spring  conrod_quat_deriv(x  y  z)

        Remove abduction, yaw, knee_spring, ankle_rod_1, ankle_rod_2, heel_spring of two legs
        2D qvel should look like like StateGeneral() in cassie2d_structs.py, but ignore the position fields

        The resulting 2D qvel should be as follows:

                0  1  2
        BASE    x  z  base_quat_deriv_Y

                3    4     5      6    7
        LEFT    hip  knee  ankle  toe  rod_quat_deriv_Y

                8    9     10     11   12
        RIGHT   hip  knee  ankle  toe  rod_quat_deriv_Y
        """
        self.qvel = np.delete(self.qvel, [1, 3, 5,
                                          6, 7, 10, 13, 14, 15, 16, 18,
                                          19, 20, 23, 26, 27, 28, 29, 31],
                              axis=1)

        """ Converting 3D torque to 2D torque
        The ordering of 3D torque as follow:

        0               1         2         3                 4
        left_abduction  left_yaw  left_hip  left_knee_spring  left_toe

        5                6          7          8                  9
        right_abduction  right_yaw  right_hip  right_knee_spring  right_toe

        Compare cassie2d_stiff.xml with cassie3d_stiff.xml
        Remove abduction and yaw actuators of two legs

        The resulting 2D torque should be as follows (see <actuator> in cassie2d_stiff.xml):

        0         1                 2         3          4                  5
        left_hip  left_knee_spring  left_toe  right_hip  right_knee_spring  right_toe
        """
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
