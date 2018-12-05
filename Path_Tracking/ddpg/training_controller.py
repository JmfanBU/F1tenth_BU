import os
import time
from collections import deque
import pickle

from ddpg import DDPG
import baselines.common.tf_util as U

import logger
import numpy as np
import tensorflow as tf
from mpi4py import MPI
import rospy

savingModelPath = '~/F1tenth_BU/Path_Tracking/log/store_NN/'

def to_angle_square(obs):
    theta = np.angle(complex(obs[0], obs[1]))
    return theta**2

def train(env, nb_epochs, nb_epoch_cycles, render_eval, reward_scale, render, param_noise, actor, critic,
    normalize_returns, normalize_observations, critic_l2_reg, actor_lr, critic_lr, action_noise,
    popart, gamma, clip_norm, nb_train_steps, nb_rollout_steps, nb_eval_steps, batch_size, memory,
    tau=0.01, eval_env=None, controller=None ,param_noise_adaption_interval=50, restore=True):
    rank = MPI.COMM_WORLD.Get_rank()

    #assert (np.abs(env.action_space.low) == env.action_space.high).all()  # we assume symmetric actions.
    assert (np.abs(env.action_space.low) == env.action_space.high).all()  # we assume symmetric actions.
    #max_action = env.action_space.high
    max_action = env.action_space.high
    logger.info('scaling actions by {} before executing in env'.format(max_action))
#    agent = DDPG(actor, critic, memory, env.observation_space.shape, env.action_space.shape,
#        gamma=gamma, tau=tau, normalize_returns=normalize_returns, normalize_observations=normalize_observations,
#        batch_size=batch_size, action_noise=action_noise, param_noise=param_noise, critic_l2_reg=critic_l2_reg,
#        actor_lr=actor_lr, critic_lr=critic_lr, enable_popart=popart, clip_norm=clip_norm,
#        reward_scale=reward_scalei)
    agent = DDPG(actor, critic, memory, env.observation_space.shape, env.action_space.shape,
        gamma=gamma, tau=tau, normalize_returns=normalize_returns, normalize_observations=normalize_observations,
        batch_size=batch_size, action_noise=action_noise, param_noise=param_noise, critic_l2_reg=critic_l2_reg,
        actor_lr=actor_lr, critic_lr=critic_lr, enable_popart=popart, clip_norm=clip_norm,
        reward_scale=reward_scale, action_range = (0., 1.))

    logger.info('Using agent with the following configuration:')
    logger.info(str(agent.__dict__.items()))

    # Set up logging stuff only for a single worker.
    if rank == 0:
        saver = tf.train.Saver(max_to_keep=600)
    else:
        saver = None

    step = 0
    episode = 0
    eval_episode_rewards_history = deque(maxlen=100)
    episode_rewards_history = deque(maxlen=100)
    with U.single_threaded_session() as sess:
        # Prepare everything.
        if restore == True:
            logger.info("Restoring from saved model")
            saver = tf.train.import_meta_graph(savingModelPath + "ddpg_test_model.meta")
            saver.restore(sess, tf.train.latest_checkpoint(savingModelPath))
        else:
            logger.info("Strarting from scratch!")
            sess.run(tf.global_variables_initializer())
        agent.initialize(sess)
        sess.graph.finalize()

        agent.reset()
        obs = env.reset()
        if eval_env is not None:
            eval_obs = eval_env.reset()
        done = False
        episode_reward = 0.
        #episode_pose = 0.
        episode_step = 0
        episodes = 0
        t = 0

        epoch = 0
        start_time = time.time()

        epoch_episode_rewards = []
        #epoch_episode_poses = []
        epoch_episode_steps = []
        epoch_episode_eval_rewards = []
        epoch_episode_eval_steps = []
        epoch_start_time = time.time()
        epoch_actions = []
        epoch_act = []
        epoch_qs = []
        epoch_episodes = 0
        for epoch in range(nb_epochs):
            for cycle in range(nb_epoch_cycles):
                # Perform rollouts.
                for t_rollout in range(nb_rollout_steps):
                    # Predict next action.
                    action, q = agent.pi(obs, apply_noise=True, compute_Q=True)
                    #assert action.shape == env.action_space.shape
                    assert action.shape == env.action_space.shape

                    # Execute next action.
                    if rank == 0 and render:
                        env.render()
                    assert max_action.shape == action.shape
                    controller.assign_param(max_action * action)
                    act = controller.control(obs)
                    #act = np.clip(act, env.action_space.low, env.action_space.high)
                    new_obs, r, done, info = env.step(act)  # scale for execution in env (as far as DDPG is concerned, every action is in [-1, 1])
                    t += 1
                    #r += -.1*(act**2)
                    #pose = to_angle_square(new_obs)
                    if rank == 0 and render:
                        env.render()
                    episode_reward += r
                    #episode_pose += pose
                    episode_step += 1

                    # Book-keeping.
                    epoch_actions.append(action)
                    epoch_act.append(act)
                    epoch_qs.append(q)
                    agent.store_transition(obs, action, r, new_obs, done)
                    obs = new_obs

                    if done:
                        # Episode done.
                        epoch_episode_rewards.append(episode_reward)
                        #epoch_episode_poses.append(episode_pose)
                        episode_rewards_history.append(episode_reward)
                        epoch_episode_steps.append(episode_step)
                        episode_reward = 0.
                        #episode_pose = 0.
                        episode_step = 0
                        epoch_episodes += 1
                        episodes += 1

                        agent.reset()
                        obs = env.reset()

                # Train.
                epoch_actor_losses = []
                epoch_critic_losses = []
                epoch_adaptive_distances = []
                for t_train in range(nb_train_steps):
                    # Adapt param noise, if necessary.
                    if memory.nb_entries >= batch_size and t_train % param_noise_adaption_interval == 0:
                        distance = agent.adapt_param_noise()
                        epoch_adaptive_distances.append(distance)

                    cl, al = agent.train()
                    epoch_critic_losses.append(cl)
                    epoch_actor_losses.append(al)
                    agent.update_target_net()

                # Evaluate.
                eval_episode_rewards = []
                #eval_episode_poses = []
                eval_qs = []
                if eval_env is not None:
                    eval_episode_reward = 0.
                    eval_episode_pose = 0.
                    for t_rollout in range(nb_eval_steps):
                        eval_action, eval_q = agent.pi(eval_obs, apply_noise=False, compute_Q=True)
                        controller.assign_param(max_action * eval_action)
                        eval_act = controller.control(eval_obs)
                        #eval_act = np.clip(eval_act, eval_env.action_space.low, eval_env.action_space.high)
                        eval_obs, eval_r, eval_done, eval_info = eval_env.step(eval_act)  # scale for execution in env (as far as DDPG is concerned, every action is in [-1, 1])
                        #eval_r += -.1*(eval_act**2)
                        #eval_p = to_angle_square(eval_obs)
                        if render_eval:
                            eval_env.render()
                        eval_episode_reward += eval_r
                        #eval_episode_pose += eval_p
                        eval_qs.append(eval_q)
                        if eval_done:
                            eval_obs = eval_env.reset()
                            eval_episode_rewards.append(eval_episode_reward)
                            #eval_episode_poses.append(eval_episode_pose)
                            eval_episode_rewards_history.append(eval_episode_reward)
                            eval_episode_reward = 0.
                            #eval_episode_pose = 0.
                if saver is not None:
                    logger.info("saving the trained model")
                    start_time_save = time.time()
                    saver.save(sess, savingModelPath + str((epoch+1)*(cycle+1)) + '/' + "ddpg_test_model")
                    logger.info('runtime saving: {}s'.format(time.time()-start_time_save))
            mpi_size = MPI.COMM_WORLD.Get_size()
            # Log stats.
            # XXX shouldn't call np.mean on variable length lists
            duration = time.time() - start_time
            stats = agent.get_stats()
            combined_stats = stats.copy()
            combined_stats['rollout/return'] = np.mean(epoch_episode_rewards)
            #combined_stats['rollout/return_pose'] = np.mean(epoch_episode_poses)
            combined_stats['rollout/return_history'] = np.mean(episode_rewards_history)
            combined_stats['rollout/episode_steps'] = np.mean(epoch_episode_steps)
            total = 0
            for params in epoch_actions:
                total += params
            total = max_action * total
            KTH_means = total[0]/len(epoch_actions)
            KE_means = total[1]/len(epoch_actions)
            #Kd_means = total[2]/len(epoch_actions)
            combined_stats['rollout/KTH_mean'] = KTH_means
            combined_stats['rollout/KE_mean'] = KE_means
            #combined_stats['rollout/Target_speed_mean'] = Kd_means
            combined_stats['rollout/Q_mean'] = np.mean(epoch_qs)
            combined_stats['train/loss_actor'] = np.mean(epoch_actor_losses)
            combined_stats['train/loss_critic'] = np.mean(epoch_critic_losses)
            combined_stats['train/param_noise_distance'] = np.mean(epoch_adaptive_distances)
            combined_stats['total/duration'] = duration
            combined_stats['total/steps_per_second'] = float(t) / float(duration)
            combined_stats['total/episodes'] = episodes
            combined_stats['rollout/episodes'] = epoch_episodes
            combined_stats['rollout/actions_std'] = np.std(epoch_actions)
            combined_stats['rollout/act_mean'] = np.mean(epoch_act)
            combined_stats['rollout/act_std'] = np.std(epoch_act)
            # Evaluation statistics.
            if eval_env is not None:
                combined_stats['eval/return'] = np.mean(eval_episode_rewards)
                #combined_stats['eval/return_pose'] = np.mean(eval_episode_poses)
                combined_stats['eval/return_history'] = np.mean(eval_episode_rewards_history)
                combined_stats['eval/Q_mean'] = np.mean(eval_qs)
                combined_stats['eval/episodes'] = len(eval_episode_rewards)
            def as_scalar(x):
                if isinstance(x, np.ndarray):
                    assert x.size == 1
                    return x[0]
                elif np.isscalar(x):
                    return x
                #else:
                #    raise ValueError('expected scalar, got %s'%x)
            combined_stats_sums = MPI.COMM_WORLD.allreduce(np.array([as_scalar(x) for x in combined_stats.values()]))
            combined_stats = {k : v / mpi_size for (k,v) in zip(combined_stats.keys(), combined_stats_sums)}

            # Total statistics.
            combined_stats['total/epochs'] = epoch + 1
            combined_stats['total/steps'] = t

            for key in sorted(combined_stats.keys()):
                logger.record_tabular(key, combined_stats[key])
            logger.dump_tabular()
            logger.info('')
            logdir = logger.get_dir()
            if rank == 0 and logdir:
                if hasattr(env, 'get_state'):
                    with open(os.path.join(logdir, 'env_state.pkl'), 'wb') as f:
                        pickle.dump(env.get_state(), f)
                if eval_env and hasattr(eval_env, 'get_state'):
                    with open(os.path.join(logdir, 'eval_env_state.pkl'), 'wb') as f:
                        pickle.dump(eval_env.get_state(), f)
