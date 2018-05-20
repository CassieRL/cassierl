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


class Cassie2dTraj(CassieTrajectory):
    def __init__(self, filepath):
        CassieTrajectory.__init__(self, filepath)

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


traj = Cassie2dTraj('../trajectory/stepdata.bin')


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
        qinit = np.array([0.0, 1.023817778, 0.039060153,
                          0.652777851, -0.023127310, -0.011201293,
                          0.442888260, 0.958797753, 0.026418149, -0.282398582, -1.570796327,
                          2.695951939, 0.002625255, -2.541116953, -2.552584648, 2.013491392,
                          0.355475724, 0.994348824, -0.006222207, -0.105870359, -1.570796327,
                          -0.661857545, 0.000059383, -0.137214229, -0.180297419, -0.741537094
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
