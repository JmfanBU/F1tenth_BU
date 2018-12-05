import argparse
import time
import os
import logging
import logger
import math
import sys
sys.path.append("~/catkin_ws/src/racecar_simulator/racecar_control/scripts/")
from rear_wheel_control_gazebo import Rear_Wheel_Path_Tracking_Feedback
from baselines import bench
from baselines.common.misc_util import (
    set_global_seeds,
    boolean_flag,
)
import training_controller as training_controller
from models import Actor, Critic
from baselines.ddpg.memory import Memory
from baselines.ddpg.noise import *

import gym
from gym import spaces
import tensorflow as tf
from mpi4py import MPI

class pid_controller:
    def __init__(self, dt):
        self.Kp_omega = 0
        self.Kd_omega = 0
        self.Ki_omega = 0
        self.integral = 0
        self.previous_v_sin = 0
        self.dt = dt

    def assign_param(self, param):
        self.integral = 0
        self.integral = 0
        self.Kp_omega = param[0]
        self.Ki_omega = param[1]
        self.Kd_omega = param[2]

    def control(self, obs):
        v_sin = obs[1]
        derivative = (v_sin - self.previous_v_sin) / self.dt
        self.integral += v_sin * self.dt
        F = (self.Kp_omega * v_sin) + (self.Kd_omega * derivative) + (self.Ki_omega * self.integral)
        self.previous_v_sin = v_sin
        return np.array([F])

    def params_space(self):
        high = np.array([200, 100, 100])
        low = np.array([-200, 100, 100])
        params = spaces.Box(low = low, high = high, dtype = np.float32)
        return params

class rear_wheel_feedback_control:

    def __init__(self):
        self.KTH = 0
        self.KE = 0

    def assign_param(self, param):
        self.KTH = param[0]
        self.KE = param[1]

    def PIDControl(self, target, current):
        a = 1.0 * (target - current)

        return a

    def control(self, obs):
        omega = obs[3] * obs[2] * math.cos(obs[1]) / (1.0 - obs[2] * obs[0]) - \
            self.KTH * abs(obs[3]) * obs[1] - self.KE * obs[3] * math.sin(obs[1]) * obs[0] / obs[1]
        if omega == 0.0 or obs[1] == 0.0:
            return np.array([0.0])
        omega = np.clip(omega, -0.7, 0.7)
        return np.array([omega])


def run(env_id, seed, noise_type, layer_norm, evaluation, **kwargs):
    # Configure things.
    rank = MPI.COMM_WORLD.Get_rank()
    if rank != 0:
        logger.set_level(logger.DISABLED)

    # Create envs.
    env = Rear_Wheel_Path_Tracking_Feedback()
    env = bench.Monitor(env, logger.get_dir() and os.path.join(logger.get_dir(), str(rank)))
    #video_train = gym.wrappers.Monitor(env, '/home/jiameng/baselines/baselines/ddpg/controller_')
    controller = rear_wheel_feedback_control()

    if evaluation and rank==0:
        eval_env = Rear_Wheel_Path_Tracking_Feedback()
        eval_env.eval_flag = True
        eval_env = bench.Monitor(eval_env, os.path.join(logger.get_dir(), 'gym_eval'))
        env = bench.Monitor(env, None)
        #video_train = gym.wrappers.Monitor(env, '/home/jiameng/baselines/baselines/ddpg/controller')
        #video_eval = gym.wrappers.Monitor(eval_env, '/home/jiameng/baselines/baselines/ddpg/controller/eval')
    else:
        eval_env = None

    # Parse noise_type
    action_noise = None
    param_noise = None
    nb_actions = env.action_space.shape[-1]
    #nb_actions = controller.params_space().shape[-1]
    for current_noise_type in noise_type.split(','):
        current_noise_type = current_noise_type.strip()
        if current_noise_type == 'none':
            pass
        elif 'adaptive-param' in current_noise_type:
            _, stddev = current_noise_type.split('_')
            param_noise = AdaptiveParamNoiseSpec(initial_stddev=float(stddev), desired_action_stddev=float(stddev))
        elif 'normal' in current_noise_type:
            _, stddev = current_noise_type.split('_')
            action_noise = NormalActionNoise(mu=np.zeros(nb_actions), sigma=float(stddev) * np.ones(nb_actions))
        elif 'ou' in current_noise_type:
            _, stddev = current_noise_type.split('_')
            action_noise = OrnsteinUhlenbeckActionNoise(mu=np.zeros(nb_actions), sigma=float(stddev) * np.ones(nb_actions))
        else:
            raise RuntimeError('unknown noise type "{}"'.format(current_noise_type))

    # Configure components.
    #memory = Memory(limit=int(1e6), action_shape=env.action_space.shape, observation_shape=env.observation_space.shape)
    memory = Memory(limit=int(1e6), action_shape=env.action_space.shape, observation_shape=env.observation_space.shape)
    critic = Critic(layer_norm=layer_norm)
    actor = Actor(nb_actions, layer_norm=layer_norm)

    # Seed everything to make things reproducible.
    seed = seed + 1000000 * rank
    logger.info('rank {}: seed={}, logdir={}'.format(rank, seed, logger.get_dir()))
    tf.reset_default_graph()
    set_global_seeds(seed)
    env.seed(seed)
    if eval_env is not None:
        eval_env.seed(seed)

    # Disable logging for rank != 0 to avoid noise.
    if rank == 0:
        start_time = time.time()
    training_controller.train(env=env, eval_env=eval_env, controller=controller, param_noise=param_noise,
        action_noise=action_noise, actor=actor, critic=critic, memory=memory, **kwargs)
    env.close()
    if eval_env is not None:
        eval_env.close()
    if rank == 0:
        logger.info('total runtime: {}s'.format(time.time() - start_time))


def parse_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--env-id', type=str, default='Path_Tracking')
    boolean_flag(parser, 'render-eval', default=False)
    boolean_flag(parser, 'layer-norm', default=True)
    boolean_flag(parser, 'render', default=False)
    boolean_flag(parser, 'normalize-returns', default=False)
    boolean_flag(parser, 'normalize-observations', default=True)
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    parser.add_argument('--critic-l2-reg', type=float, default=1e-2)
    parser.add_argument('--batch-size', type=int, default=64)  # per MPI worker
    parser.add_argument('--actor-lr', type=float, default=1e-4)
    parser.add_argument('--critic-lr', type=float, default=1e-3)
    boolean_flag(parser, 'popart', default=False)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--reward-scale', type=float, default=1.)
    parser.add_argument('--clip-norm', type=float, default=None)
    parser.add_argument('--nb-epochs', type=int, default=50)  # with default settings, perform 1M steps total
    parser.add_argument('--nb-epoch-cycles', type=int, default=10)
    parser.add_argument('--nb-train-steps', type=int, default=20)  # per epoch cycle and MPI worker
    parser.add_argument('--nb-eval-steps', type=int, default=100)  # per epoch cycle and MPI worker
    parser.add_argument('--nb-rollout-steps', type=int, default=100)  # per epoch cycle and MPI worker
    parser.add_argument('--noise-type', type=str, default='adaptive-param_0.2')  # choices are adaptive-param_xx, ou_xx, normal_xx, none
    parser.add_argument('--num-timesteps', type=int, default=None)
    boolean_flag(parser, 'evaluation', default=False)
    args = parser.parse_args()
    # we don't directly specify timesteps for this script, so make sure that if we do specify them
    # they agree with the other parameters
    if args.num_timesteps is not None:
        assert(args.num_timesteps == args.nb_epochs * args.nb_epoch_cycles * args.nb_rollout_steps)
    dict_args = vars(args)
    del dict_args['num_timesteps']
    return dict_args


if __name__ == '__main__':
    args = parse_args()
    if MPI.COMM_WORLD.Get_rank() == 0:
        logger.configure()
    # Run actual script.
    run(**args)
