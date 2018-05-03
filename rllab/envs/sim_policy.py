"""
This program runs the trained policy on the robot by loading the parameter file
param.pkl.
"""

import argparse
import cassie2d as cassie
from rllab.envs.normalized_env import normalize


import joblib
import tensorflow as tf

from rllab.misc.console import query_yes_no
from rllab.sampler.utils import rollout

if __name__ == "__main__":

    filename = "params.pkl"
    data = joblib.load(filename)

    policy = data['policy']
    env = normalize(cassie.Cassie2dEnv())

    max_path_length = 10000
    speedup = 1


    while True:
        path = rollout(env, policy, max_path_length=max_path_length,
                       animated=False, speedup=speedup)
