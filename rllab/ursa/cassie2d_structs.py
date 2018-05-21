import ctypes as ct
import numpy as np


class ControllerTorque(ct.Structure):
    _fields_ = [('torques', ct.c_double*6)]


class ControllerForce(ct.Structure):
    _fields_ = [('left_force', ct.c_double*3),
                ('right_force', ct.c_double*3)]


class ControllerOsc(ct.Structure):
    """OSC = Operational Space Control"""

    _fields_ = [('body_xdd', ct.c_double*2),
                ('left_xdd', ct.c_double*2),
                ('right_xdd', ct.c_double*2),
                ('pitch_add', ct.c_double)]


class ControllerPd(ct.Structure):
    """PD = Target Joint Angles"""

    _fields_= [('angles', ct.c_double * 6)]


class StateGeneral(ct.Structure):
    """
    arrays of size 3 consist of x, z, phi
    arrays of size 5 consist of hip, knee, ankle, toe, rod
    """

    _fields_ = [('base_pos', ct.c_double*3),
                ('base_vel', ct.c_double*3),
                ('left_pos', ct.c_double*5),
                ('left_vel', ct.c_double*5),
                ('right_pos', ct.c_double*5),
                ('right_vel', ct.c_double*5)]


class StateOperationalSpace(ct.Structure):
    """This class has the same structure as StateGeneral."""

    _fields_ = [('body_x', ct.c_double*3),
                ('body_xd', ct.c_double*3),
                ('left_x', ct.c_double*3),
                ('left_xd', ct.c_double*3),
                ('right_x', ct.c_double*3),
                ('right_xd', ct.c_double*3)]


class InterfaceStructConverter():
    """Converting arrays to structures and structures to arrays"""

    def operational_state_to_array(self, state):
        s = np.zeros((18,), dtype=np.double)
        for i in range(3):
            s[i] = state.body_x[i]
            s[i+3] = state.body_xd[i]
            s[i+6] = state.left_x[i]
            s[i+9] = state.left_xd[i]
            s[i+12] = state.right_x[i]
            s[i+15] = state.right_xd[i]
        return s

    # To make feet position wrt body, and not in global frame
    def operational_state_array_to_pos_invariant_array(self, state_array):
        s = np.zeros((17,), dtype=np.double)
        for i in range(17):
            s[i] = state_array[i+1]
        s[5] -= state_array[0]     # substracting x-position to make position invariant, left foot (I guess)
        s[11] -= state_array[0]    # right foot (I guess)
        return s

    def general_state_to_array(self, state):
        s = np.zeros((26,), dtype=np.double)
        for i in range(3):
            s[i] = state.base_pos[i]
            s[i+3] = state.base_vel[i]
        for i in range(5):
            s[i+6] = state.left_pos[i]
            s[i+11] = state.left_vel[i]
            s[i+16] = state.right_pos[i]
            s[i+21] = state.right_vel[i]
        return s

    def array_to_general_state(self, s):
        state = StateGeneral()
        for i in range(3):
            state.base_pos[i] = s[i]
            state.base_vel[i] = s[i+3]
        for i in range(5):
            state.left_pos[i] = s[i+6]
            state.left_vel[i] = s[i+11]
            state.right_pos[i] = s[i+16]
            state.right_vel[i] = s[i+21]
        return state

    def array_to_operational_action(self, action):
        a = ControllerOsc()
        for i in range(2):
            a.body_xdd[i] = action[i]
            a.left_xdd[i] = action[2+i]
            a.right_xdd[i] = action[4+i]
        a.pitch_add = action[6]
        return a

    # add conversion for torque
    def array_to_torque_action(self, action):
        a = ControllerTorque()
        for i in range(6):
            a.torques[i] = action[i]
        return a

    # add conversion for target join angles
    def array_to_pd_action(self, action):
        a = ControllerPd()
        for i in range(6):
            a.angles[i] = action[i]
        return a
