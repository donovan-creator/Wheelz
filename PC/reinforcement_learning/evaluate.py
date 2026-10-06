"""Held-out simulation evaluation with explicit stopping and failure metrics."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO

from robot_env import WheelzGoalEnv, wrap_angle

HERE = Path(__file__).resolve().parent
DEFAULT_RUN = HERE / 'runs' / 'discrete_ppo_v2'


def heuristic(obs, config):
    distance = float(obs[8]) * 2 * config['task']['world_half_extent_m']
    if distance <= config['task']['goal_tolerance_m']:
        return 0
    bearing = math.atan2(float(obs[2]), float(obs[3]))
    reverse = abs(bearing) > math.pi / 2
    error = wrap_angle(bearing - math.pi) if reverse else bearing
    yaw_speed = float(obs[5]) * 2 * config['robot']['max_wheel_speed_mps'] / config['robot']['track_width_m']
    error -= yaw_speed * config['robot']['motor_time_constant_s']
    if abs(error) > .18:
        return 3 if error > 0 else 4
    return 2 if reverse else 1


def evaluate(policy, config_path, episodes, seed, randomized=True, watch=False):
    env = WheelzGoalEnv(config_path, render_mode='human' if watch else None,
                       domain_randomization=randomized)
    records = []
    counts = np.zeros(5, dtype=int)
    try:
        for episode in range(episodes):
            obs, _ = env.reset(seed=seed + episode)
            total_reward = 0.0
            for step in range(env.max_steps):
                action = int(policy(obs, env.cfg))
                counts[action] += 1
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                if terminated or truncated:
                    break
            records.append({'seed': seed + episode, 'success': bool(info['is_success']),
                            'out_of_bounds': bool(info['out_of_bounds']),
                            'timeout': bool(truncated and not terminated),
                            'final_distance_m': float(info['distance_to_goal']),
                            'seconds': (step + 1) * env.dt, 'return': total_reward})
            if watch:
                print(records[-1])
    finally:
        env.close()
    successes = sum(row['success'] for row in records)
    rate = successes / episodes
    # Wilson 95% interval (binomial episodes).
    z = 1.96
    denominator = 1 + z * z / episodes
    center = (rate + z * z / (2 * episodes)) / denominator
    half = z * math.sqrt(rate * (1 - rate) / episodes + z * z / (4 * episodes**2)) / denominator
    return {
        'episodes': episodes, 'seed_start': seed, 'randomized': randomized,
        'successes': successes, 'success_rate': rate,
        'success_95pct_interval': [center - half, center + half],
        'out_of_bounds': sum(r['out_of_bounds'] for r in records),
        'timeouts': sum(r['timeout'] for r in records),
        'mean_final_distance_m': float(np.mean([r['final_distance_m'] for r in records])),
        'mean_episode_seconds': float(np.mean([r['seconds'] for r in records])),
        'mean_return': float(np.mean([r['return'] for r in records])),
        'action_counts': counts.tolist(), 'records': records,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=DEFAULT_RUN / 'best' / 'best_model.zip')
    parser.add_argument('--config', type=Path, default=DEFAULT_RUN / 'robot_config.json')
    parser.add_argument('--episodes', type=int, default=200)
    parser.add_argument('--seed', type=int, default=200_000)
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('--nominal', action='store_true')
    parser.add_argument('--baselines', action='store_true')
    parser.add_argument('--output', type=Path, help='JSON report; watching does not overwrite evaluation by default')
    args = parser.parse_args()
    if args.episodes <= 0:
        parser.error('--episodes must be positive')
    torch.set_num_threads(1)
    model = PPO.load(args.model, device='cpu')
    if model.action_space.n != 5:
        raise ValueError('Not a Wheelz five-action checkpoint')
    result = {'model': str(args.model), 'ppo': evaluate(
        lambda obs, cfg: model.predict(obs, deterministic=True)[0], args.config,
        args.episodes, args.seed, not args.nominal, args.watch)}
    if args.baselines:
        random = np.random.default_rng(args.seed)
        result['random'] = evaluate(lambda obs, cfg: random.integers(5), args.config,
                                    args.episodes, args.seed, not args.nominal)
        result['heuristic'] = evaluate(heuristic, args.config, args.episodes,
                                       args.seed, not args.nominal)
    output = args.output if args.output is not None else (None if args.watch else DEFAULT_RUN / 'evaluation.json')
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    for name, entry in result.items():
        if isinstance(entry, dict):
            print(f"{name}: {entry['successes']}/{entry['episodes']} settled goals, "
                  f"success={entry['success_rate']:.1%}, "
                  f"mean distance={entry['mean_final_distance_m']:.3f}m")
    if output is not None:
        print(f'Full evaluation: {output}')


if __name__ == '__main__':
    main()
