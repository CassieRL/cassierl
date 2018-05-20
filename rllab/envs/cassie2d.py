import ctypes
import numpy as np
import time

from cached_property import cached_property
from cassie2d_structs import ControllerForce
from cassie2d_structs import ControllerOsc
from cassie2d_structs import ControllerPd
from cassie2d_structs import ControllerTorque
from cassie2d_structs import InterfaceStructConverter as convert
from cassie2d_structs import StateGeneral
from cassie2d_structs import StateOperationalSpace
from cassie2d_trajectory import Cassie2dTraj
from ctypes import cdll
from rllab.envs.base import Env
from rllab.envs.base import Step
from rllab.misc import logger
from rllab.misc.overrides import overrides
from rllab.spaces import Box

trajectory = Cassie2dTraj('../trajectory/stepdata.bin')
lib = cdll.LoadLibrary('../../bin/libcassie2d.so')
c_double_p = ctypes.POINTER(ctypes.c_double)

lib.Cassie2dInit.argtypes = None
lib.Cassie2dInit.restype = ctypes.c_void_p

lib.Reset.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateGeneral)]
lib.Reset.restype = None

lib.StepOsc.argtypes = [ctypes.c_void_p, ctypes.POINTER(ControllerOsc)]
lib.StepOsc.restype = None

lib.StepJacobian.argtypes = [ctypes.c_void_p, ctypes.POINTER(ControllerForce)]
lib.StepJacobian.restype = None

lib.StepTorque.argtypes = [ctypes.c_void_p, ctypes.POINTER(ControllerTorque)]
lib.StepTorque.restype = None

lib.StepPd.argtypes = [ctypes.c_void_p, ctypes.POINTER(ControllerPd)]
lib.StepPd.restype = None

lib.GetGeneralState.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateGeneral)]
lib.GetGeneralState.restype = None

lib.GetOperationalSpaceState.argtypes = [ctypes.c_void_p, ctypes.POINTER(StateOperationalSpace)]
lib.GetOperationalSpaceState.restype = None

lib.Display.argtypes = [ctypes.c_void_p, ctypes.c_bool]
lib.Display.restype = None

control_mode = 'PD'
assert control_mode == 'OSC' or control_mode == 'Torque' or control_mode == 'PD', 'Invalid Control Mode'

print('Control mode = ' + control_mode)

t = 0.0

class Cassie2dEnv(Env):
    """
    Modifies the step() method to make Cassie learn what you want.
    """

    def __init__(self):
        self.qstate = StateGeneral()
        self.xstate = StateOperationalSpace()

        self.action_osc = ControllerOsc()
        self.action_jac = ControllerForce()
        self.action_tor = ControllerTorque()
        self.action_pd = ControllerPd()

        self.cassie = lib.Cassie2dInit()
        self.cvrt = convert()

        lib.Display(self.cassie, True)

    def reset(self):
        qinit = np.array([0.0, 0.939, 0.0,
                          0.0, 0.0, 0.0,
                          0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407,
                          0.0, 0.0, 0.0, 0.0, 0.0,
                          0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407,
                          0.0, 0.0, 0.0, 0.0, 0.0
                          ], dtype=ctypes.c_double)
        self.qstate = self.cvrt.array_to_general_state(qinit)
        lib.Reset(self.cassie, self.qstate)

        # current state
        lib.GetOperationalSpaceState(self.cassie, self.xstate)
        s = self.cvrt.operational_state_to_array(self.xstate)

        return self.cvrt.operational_state_array_to_pos_invariant_array(s)

    def step(self, action, n=10):
        """
        Args:
            n   When you tell RL you did 1 step, you actually did n steps.
                This acts like a frequency parameter to run your policy.
        """
        # convert action
        if control_mode == 'OSC':
            self.action_osc = self.cvrt.array_to_operational_action(action)
        elif control_mode == 'Torque':
            self.action_tor = self.cvrt.array_to_torque_action(action)
        elif control_mode == 'PD':
            self.action_pd = self.cvrt.array_to_pd_action(action)

        # current state
        lib.GetOperationalSpaceState(self.cassie, self.xstate)

        # run the simulation forward by sending 'action'
        for _ in range(n):
            if control_mode == 'OSC':
                lib.StepOsc(self.cassie, self.action_osc)
            elif control_mode == 'Torque':
                lib.StepTorque(self.cassie, self.action_tor)
            elif control_mode == 'PD':
                lib.StepPd(self.cassie, self.action_pd)
            t += 0.0005

        # next state
        lib.GetGeneralState(self.cassie, self.qstate)
        lib.GetOperationalSpaceState(self.cassie, self.xstate)
        sp = self.cvrt.operational_state_to_array(self.xstate)
        sp = self.cvrt.operational_state_array_to_pos_invariant_array(sp)

        """ --------------------------REWARD FUNCTION--------------------------
        defined by
            r = w_joint*r_joint + w_rp*r_rp + w_ro*r_ro + w_spring*r_spring
        where
            w_joint     weight for r_joint, recommended 0.5
            r_joint     measures how similar the active joint angles are to the reference motion
                        and is equal to exp[-(x_joint - x_ref_joint)^2]

            w_rp        weight for r_rp, recommended 0.3
            r_rp        measures how similar the pelvis position are to the reference motion

            w_ro        weight for r_ro, recommended 0.1
            r_ro        measures how similar the pelvis orientation are to the reference motion

            w_spring    eight for r_spring, recommended 0.1
            r_spring    an additional term to help stabilize the springs on the shin joints
        """

        # sample trajectory
        ref_qpos, _ = trajectory.state(t)

        # append space for trajectory on observation then use GetGeneralState to fill values
        # sp[17] = base_x, sp[18] = base_z, sp[19] = base_phi
        # sp[20] = left_hip, sp[21] = left_knee, sp[22] = left_toe
        # sp[23] = right_hip, sp[24] = right_knee, sp[25] = right_toe
        sp = np.append(sp, [ref_qpos[0], ref_qpos[1], ref_qpos[2],
                            ref_qpos[3], ref_qpos[4], ref_qpos[6],
                            ref_qpos[8], ref_qpos[9], ref_qpos[11]])

        # weights
        w_joint = 0.5
        w_pelvis_position = 0.3
        w_pelvis_orientation = 0.1

        # joint positions: 0 = hip, 1 = knee, 3 = toe (see StateGeneral() in cassie2d_structs.py)
        j = self.qstate.left_pos[0] + self.qstate.left_pos[1] + self.qstate.left_pos[3]
        j += self.qstate.right_pos[0] + self.qstate.right_pos[1] + self.qstate.right_pos[3]
        j -= sum(sp[20:])
        j = np.exp(-(j**2))

        # pelvis position: 0 = x, 1 = z (see StateGeneral() in cassie2d_structs.py)
        p = self.xstate.body_x[0] + self.xstate.body_x[1]
        p -= sum(sp[17:19])
        p = np.exp(-(p**2))

        # pelvis orientation: 2 = phi (see StateGeneral() in cassie2d_structs.py)
        o = self.xstate.body_x[2]
        o -= sp[19]  # reference position for phi
        o = np.exp(-(o**2))

        # finalize the reward function
        r = w_joint*j + w_pelvis_position*p + w_pelvis_orientation*o

        # termination
        done = False
        if (self.xstate.body_x[1] < 0.6) or (self.xstate.body_x[1] > 1.2) or (r < 0.6):
            done = True

        return Step(observation=sp, reward=r, done=done)

    def render(self):
        lib.Render(self.cassie)
        pass

    ####################################################################################################################
    #                     Controller to step through a preload trajectory and export to a CSV file
    ####################################################################################################################
    def step_traj_export_csv(self, t):
        qpos, qvel = trajectory.state(t)
        torques = trajectory.action(t)[2]

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

    ####################################################################################################################
    #                             THE CONTROLLERS BELOW ARE FOR SQUATTING, NOT FOR RL
    ####################################################################################################################

    def standing_controller_osc(self, zpos_target, zvel_target):
        # get operational state
        lib.GetOperationalSpaceState(self.cassie, ctypes.byref(self.xstate))

        stance_kp = 100.0

        # first we want to hold both feet on the ground
        self.action_osc.left_xdd[0] = 0.0
        self.action_osc.left_xdd[1] = stance_kp*(-5e-3 - self.xstate.left_x[1])
        self.action_osc.right_xdd[0] = 0.0
        self.action_osc.right_xdd[1] = stance_kp * \
            (-5e-3 - self.xstate.right_x[1])

        # calculate desired body x position based on average foot pos
        xpos_target = (self.xstate.left_x[0] + self.xstate.right_x[0])/2.0
        xvel_target = 0.0

        com_kp = 100.0
        com_kd = 20.0

        self.action_osc.body_xdd[0] = com_kp*(xpos_target - self.xstate.body_x[0]) + com_kd*(
            xvel_target - self.xstate.body_xd[0])
        self.action_osc.body_xdd[1] = com_kp*(zpos_target - self.xstate.body_x[1]) + com_kd*(
            zvel_target - self.xstate.body_xd[1])

        # last apply a small effort to keep pelvis from drifting too far from 0.0
        pitch_kp = 20.0
        pitch_kd = 10.0
        self.action_osc.pitch_add = pitch_kp * \
            (0.0 - self.xstate.body_x[2]) + \
            pitch_kd*(0.0 - self.xstate.body_xd[2])

        lib.StepOsc(self.cassie, ctypes.byref(self.action_osc))

    def standing_controller_jacobian(self, zpos_target, zvel_target):
        # get operational state
        lib.GetOperationalSpaceState(self.cassie, ctypes.byref(self.xstate))

        # calculate desired body x position based on average foot pos
        xpos_target = (self.xstate.left_x[0] + self.xstate.right_x[0])/2.0
        xvel_target = 0.0

        com_kp = 200.0
        com_kd = 50.0

        self.action_jac.left_force[0] = com_kp*(
            xpos_target - self.xstate.body_x[0]) + com_kd*(xvel_target - self.xstate.body_xd[0])
        self.action_jac.right_force[0] = self.action_jac.left_force[0]
        self.action_jac.left_force[1] = 0.5*9.806*31.0 + com_kp*(
            zpos_target - self.xstate.body_x[1]) + com_kd*(zvel_target - self.xstate.body_xd[1])
        self.action_jac.right_force[1] = self.action_jac.left_force[1]

        # last apply a small effort to keep pelvis from drifting too far from 0.0
        pitch_kp = 100.0
        pitch_kd = 10.0
        self.action_jac.left_force[2] = pitch_kp*(
            0.0 - self.xstate.body_x[2]) + pitch_kd*(0.0 - self.xstate.body_xd[2])
        self.action_jac.right_force[2] = self.action_jac.left_force[2]

        if self.action_jac.left_force[1] < 0.0:
            self.action_jac.left_force[1] = 0.0
        if self.action_jac.right_force[1] < 0.0:
            self.action_jac.right_force[1] = 0.0

        # for i in range(3):
        #    self.action_jac.left_force[i] = -1.0*self.action_jac.left_force[i]
        #    self.action_jac.right_force[i] = -1.0*self.action_jac.right_force[i]

        lib.StepJacobian(self.cassie, ctypes.byref(self.action_jac))

    ####################################################################################################################
    #                         Functions needed for RLLAB action space and observation space
    ####################################################################################################################

    @cached_property
    def observation_space(self):
        high = np.full((26,), 1e20)
        low = -high
        return Box(low, high)

    @cached_property
    def action_space(self):
        """
        The order of these numbers follows the order of the children in the
        <actuator> tag defined in the XML file.

        For example, in cassie2d_stiff.xml, the order of the actuators are
            1. left_hip
            2. left_knee_spring
            3. left_toe
            4. right_hip
            5. right_knee_spring
            6. right_toe
        """
        if control_mode == 'OSC':
            high = np.full((7,), 2e1)  # accel of 100 may be reasonable
            low = np.array([-2e1, -2e1, -2e1, 0, -2e1, 0, -2e1])
        elif control_mode == 'Torque':
            high = np.array([12.0, 12.0, 0.9, 12.0, 12.0, 0.9])
            low = -1.0*high
        elif control_mode == 'PD':
            high = np.array([np.radians(80.0), np.radians(-37.0), np.radians(-30.0),
                             np.radians(80.0), np.radians(-37.0), np.radians(-30.0)])
            low = np.array([np.radians(-50.0), np.radians(-164.0), np.radians(-140.0),
                            np.radians(-50.0), np.radians(-164.0), np.radians(-140.0)])
        return Box(low, high)

    def terminate(self):
        print('Process terminated.')
        del self.cassie
