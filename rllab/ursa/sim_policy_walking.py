"""
This program runs the trained policy on the robot by loading the parameter file
param.pkl.
"""

import argparse
import cassie2d as cassie
import joblib
import tensorflow as tf

from rllab.envs.normalized_env import normalize
from rllab.misc.console import query_yes_no
from rllab.sampler.utils import rollout

if __name__ == '__main__':
    filename = 'params_walking_experiment_2018_05_13_23_15_55_0001.pkl'
    data = joblib.load(filename)

    policy = data['policy']
    env = normalize(cassie.Cassie2dEnv(
        control_mode='PD',
        motion_mode='Walking'
    ))

    max_path_length = 10000
    speedup = 50

    while True:
        path = rollout(env, policy, max_path_length=max_path_length,
                       animated=True, speedup=speedup)
