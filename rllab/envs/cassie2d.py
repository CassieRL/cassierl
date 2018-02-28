import ctypes
from ctypes import cdll
import numpy as np
from cassie2d_structs import ControllerOsc
from cassie2d_structs import ControllerForce
from cassie2d_structs import ControllerTorque
from cassie2d_structs import StateGeneral
from cassie2d_structs import StateOperationalSpace
from cassie2d_structs import InterfaceStructConverter as convert
from rllab.envs.base import Env
from rllab.envs.base import Step
from rllab.spaces import Box
from cached_property import cached_property
import time

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

lib.GetGeneralState.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateGeneral)]
lib.GetGeneralState.restype = None

lib.GetOperationalSpaceState.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateOperationalSpace)]
lib.GetOperationalSpaceState.restype = None

lib.Display.argtypes = [ctypes.c_void_p, ctypes.c_bool]
lib.Display.restype = None

control_mode = {"torque" : 0, "jacobian" : 1, "operational" : 2}

class Cassie2dEnv(Env):
    """
    Modifies the step() method to make Cassie learn what you want.
    """

    def __init__(self):
        self.qstate = StateGeneral()
        self.xstate = StateOperationalSpace()
        self.action_osc = ControllerOsc()
        self.action_jac = ControllerForce()
        self.cassie = lib.Cassie2dInit()
        self.cvrt = convert()
        lib.Display(self.cassie, True)

    def reset(self):
        qinit = np.array([0.0, 0.939, 0.0, 0.0, 0.0, 0.0, 0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407, 0.0, 0.0, 0.0, 0.0, 0.0, 0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=ctypes.c_double)
        self.qstate = self.cvrt.array_to_general_state(qinit)
        lib.Reset(self.cassie, self.qstate)

    def step(self, action):

        #convert action
        self.action_osc = self.cvrt.array_to_operational_action(action)

        #current state
        lib.GetOperationalSpaceState(self.cassie, self.xstate)
        s = self.cvrt.operational_state_to_array(self.xstate)

        lib.StepOsc(self.cassie, self.action_osc)

        #next state
        lib.GetOperationalSpaceState(self.cassie, self.xstate)
        sp = self.cvrt.operational_state_to_array(self.xstate)

        #reward (do something here)
        r=0.0
        #done (do something here)
        done=False

        return Step(observation=sp, reward=r, done=done)

    def standing_controller_osc(self, zpos_target, zvel_target):

        #get operational state
        lib.GetOperationalSpaceState(self.cassie, ctypes.byref(self.xstate))

        stance_kp = 100.0

        #first we want to hold both feet on the ground
        self.action_osc.left_xdd[0] = 0.0;
        self.action_osc.left_xdd[1] = stance_kp*(-5e-3 - self.xstate.left_x[1]);
        self.action_osc.right_xdd[0] = 0.0;
        self.action_osc.right_xdd[1] = stance_kp*(-5e-3 - self.xstate.right_x[1]);

        #calculate desired body x position based on average foot pos
        xpos_target = (self.xstate.left_x[0] + self.xstate.right_x[0])/2.0
        xvel_target = 0.0

        com_kp = 100.0
        com_kd = 20.0

        self.action_osc.body_xdd[0] = com_kp*(xpos_target - self.xstate.body_x[0]) + com_kd*(xvel_target - self.xstate.body_xd[0])
        self.action_osc.body_xdd[1] = com_kp*(zpos_target - self.xstate.body_x[1]) + com_kd*(zvel_target - self.xstate.body_xd[1])

        #last apply a small effort to keep pelvis from drifting too far from 0.0
        pitch_kp = 20.0
        pitch_kd = 10.0
        self.action_osc.pitch_add = pitch_kp*(0.0 - self.xstate.body_x[2]) + pitch_kd*(0.0 - self.xstate.body_xd[2])

        lib.StepOsc(self.cassie, ctypes.byref(self.action_osc))

    def standing_controller_jacobian(self, zpos_target, zvel_target):

        #get operational state
        lib.GetOperationalSpaceState(self.cassie, ctypes.byref(self.xstate))

        #calculate desired body x position based on average foot pos
        xpos_target = (self.xstate.left_x[0] + self.xstate.right_x[0])/2.0
        xvel_target = 0.0

        com_kp = 200.0
        com_kd = 50.0

        self.action_jac.left_force[0] = com_kp*(xpos_target - self.xstate.body_x[0]) + com_kd*(xvel_target - self.xstate.body_xd[0])
        self.action_jac.right_force[0] = self.action_jac.left_force[0]
        self.action_jac.left_force[1] = 0.5*9.806*31.0 + com_kp*(zpos_target - self.xstate.body_x[1]) + com_kd*(zvel_target - self.xstate.body_xd[1])
        self.action_jac.right_force[1] = self.action_jac.left_force[1]

        #last apply a small effort to keep pelvis from drifting too far from 0.0
        pitch_kp = 100.0
        pitch_kd = 10.0
        self.action_jac.left_force[2] = pitch_kp*(0.0 - self.xstate.body_x[2]) + pitch_kd*(0.0 - self.xstate.body_xd[2])
        self.action_jac.right_force[2] = self.action_jac.left_force[2]

        if self.action_jac.left_force[1] < 0.0:
            self.action_jac.left_force[1] = 0.0
        if self.action_jac.right_force[1] < 0.0:
            self.action_jac.right_force[1] = 0.0

        # for i in range(3):
        #    self.action_jac.left_force[i] = -1.0*self.action_jac.left_force[i]
        #    self.action_jac.right_force[i] = -1.0*self.action_jac.right_force[i]

        lib.StepJacobian(self.cassie, ctypes.byref(self.action_jac))

    @cached_property
    def observation_space(self):
        high = np.full((18,), 1e20)
        low = -high
        return Box(low, high)


    @cached_property
    def action_space(self):
        high = np.full((6,), 1e2) #accel of 100 may be reasonable
        low = -high
        return Box(low, high)

def main():
    env = Cassie2dEnv()
    env.reset()

    freq = 0.5
    w = freq*3.1415
    t = 0.0

    while (True):
        env.standing_controller_jacobian(0.7 + 0.25*np.sin(w*t), 0.25*np.cos(w*t))
        t = t + 0.0005




if __name__ == "__main__":
    main()
