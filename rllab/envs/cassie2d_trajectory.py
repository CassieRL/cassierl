import ctypes
import math
import numpy as np
import random
import time

from cached_property import cached_property
from cassie2d_structs import ControllerForce
from cassie2d_structs import ControllerOsc
from cassie2d_structs import ControllerPd
from cassie2d_structs import ControllerTorque
from cassie2d_structs import InterfaceStructConverter as convert
from cassie2d_structs import StateGeneral
from cassie2d_structs import StateOperationalSpace
from ctypes import cdll
from rllab.envs.base import Env
from rllab.envs.base import Step
from rllab.misc import logger
from rllab.misc.overrides import overrides
from rllab.spaces import Box

lib = cdll.LoadLibrary('../../bin/libcassie2d.so')
c_double_p = ctypes.POINTER(ctypes.c_double)

lib.Cassie2dInit.argtypes = None
lib.Cassie2dInit.restype = ctypes.c_void_p

lib.Reset.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateGeneral)]
lib.Reset.restype = None

lib.StepOsc.argtypes = [ctypes.c_void_p,ctypes.POINTER(ControllerOsc)]
lib.StepOsc.restype = None

lib.StepJacobian.argtypes = [ctypes.c_void_p,ctypes.POINTER(ControllerForce)]
lib.StepJacobian.restype = None

lib.StepTorque.argtypes = [ctypes.c_void_p,ctypes.POINTER(ControllerTorque)]
lib.StepTorque.restype = None

lib.StepPd.argtypes = [ctypes.c_void_p,ctypes.POINTER(ControllerPd)]
lib.StepPd.restype = None

lib.GetGeneralState.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateGeneral)]
lib.GetGeneralState.restype = None

lib.GetOperationalSpaceState.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateOperationalSpace)]
lib.GetOperationalSpaceState.restype = None

lib.Display.argtypes = [ctypes.c_void_p, ctypes.c_bool]
lib.Display.restype = None

print('Control mode = PD')


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


traj = CassieTrajectory2d('../trajectory/stepdata.bin')


class Cassie2dEnv(Env):
    """
    Modifies the step() method to make Cassie learn what you want.
    """

    def __init__(self):
        self.qstate = StateGeneral()
        self.xstate = StateOperationalSpace()

        self.action_tor = ControllerTorque()
        self.action_pd = ControllerPd()

        self.cassie = lib.Cassie2dInit()
        self.cvrt = convert()

        lib.Display(self.cassie, True)

    def reset(self):
        qinit = np.array([0.0, 1.023818, 0.03906,
                          0.652778, -0.023127, -0.05085,
                          0.442888, 0.958798, 0.026418, -0.282399, 1.570796,
                          0.002625, 0.3350840, -2.552585, -0.017491, -1.570796,
                          0.035659, -0.013044, 0.994349, 0.004478, 1.570796,
                          -0.661858, 0.000059, -0.1372140, -0.1802970, 0.114799
                         ], dtype=ctypes.c_double)
        self.qstate = self.cvrt.array_to_general_state(qinit)
        lib.Reset(self.cassie, self.qstate)

        # current state
        lib.GetOperationalSpaceState(self.cassie, self.xstate)
        s = self.cvrt.operational_state_to_array(self.xstate)

        return self.cvrt.operational_state_array_to_pos_invariant_array(s)

    def step(self, t):
        qpos, qvel = traj.state(t)
        torques = traj.action(t)[2]

        kp = 10.0
        kd = 5.0
        joints = [3, 4, 6, 8, 9, 11]
        angles = []

        for i in range(len(joints)):
            angles.append((torques[i] - kd*(0.0 - qvel[joints[i]]))/kp + qpos[joints[i]])

        with open('trajectory.csv', 'a') as f:
            f.write(str(t) + ',')
            for i in range(len(angles)):
                f.write(str(angles[i]) + ',')
            for i in range(len(joints)):
                f.write(str(qvel[joints[i]]))
                if i < len(joints) - 1:
                    f.write(',')
            f.write('\n')

        self.action_pd = self.cvrt.array_to_pd_action(angles)
        lib.StepPd(self.cassie, ctypes.byref(self.action_pd))

    def render(self):
        lib.Render(self.cassie)
        pass
