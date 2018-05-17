from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from cassie2d import Cassie2dEnv
from rllab.envs.normalized_env import normalize
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.misc.instrument import run_experiment_lite
import joblib
from rllab.algos.vpg import VPG


load_policy = False


def run_task(*_):
    env = normalize(Cassie2dEnv())

    if load_policy:
        filename = '123'
        data = joblib.load(filename)
        policy = data['policy']
        print('Loading Pretrained Policy ...............................')
    else:
        # use Gaussian maximum likelihood policy
        policy = GaussianMLPPolicy(
            env_spec=env.spec,
            # The neural network policy should have two hidden layers, each with 128 hidden units.
            hidden_sizes=(128, 128),
            init_std=1.0,
            # adaptive_std=True,
        )

    baseline = LinearFeatureBaseline(env_spec=env.spec)

    algo = VPG(
        env=env,
        policy=policy,
        baseline=baseline,
        batch_size=10000,
        max_path_length=1000,  # dt = (1/2000)*n, where n is Step(n)
        n_itr=1000,
        discount=0.99,
        step_size=0.005,      # default was 0.01
        # Uncomment both lines (this and the plot parameter below) to enable plotting
        plot=True,
        use_gpu=True
    )
    algo.train()


run_experiment_lite(
    run_task,
    # Number of parallel workers for sampling
    n_parallel=1,
    # Only keep the snapshot parameters for the last iteration
    snapshot_mode='last',
    # Specifies the seed for the experiment. If this is not provided, a random seed
    # will be used
    seed=1,
    plot=True,
    use_gpu=True
)
