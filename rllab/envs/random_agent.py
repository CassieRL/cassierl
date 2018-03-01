"""
Adapted from OpenAI Gym's random_agent.py.
This thing will never work unless you're lucky.
"""

import argparse
import sys
from cassie2d import Cassie2dEnv

import gym
from gym import wrappers, logger


def main():
    env = Cassie2dEnv()
    env.reset()

    agent = RandomAgent(env.action_space)
    episode_count = 100
    reward = 0
    done = False

    for episode in range(episode_count):
        ob = env.reset()
        while True:
            action = agent.act(ob, reward, done)
            ob, reward, done, info = env.step(action)
            print(episode)
            env.render()
            if done:
                break


class RandomAgent(object):
    """The world's simplest agent!"""

    def __init__(self, action_space):
        self.action_space = action_space

    def act(self, observation, reward, done):
        return self.action_space.sample()


if __name__ == '__main__':
    main()
