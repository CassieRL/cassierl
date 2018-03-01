"""
Adapted from OpenAI Gym's random_agent.py.
This thing will never work unless you're lucky.
"""

from cassie2d import Cassie2dEnv


def main():
    env = Cassie2dEnv()
    env.reset()

    agent = RandomAgent(env.action_space)
    episode_count = 100
    reward = 0
    done = False

    for episode in range(episode_count):
        ob = env.reset()
        print("Current episode: %d" % episode)

        while True:
            action = agent.act(ob, reward, done)
            ob, reward, done, _ = env.step(action)
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
