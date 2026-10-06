"""Save deterministic held-out example trajectories as a standalone report."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from stable_baselines3 import PPO
from robot_env import WheelzGoalEnv

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, default=HERE / 'runs/discrete_ppo_v2')
    parser.add_argument('--output', type=Path, default=HERE / 'runs/discrete_ppo_v2/trajectories.png')
    args = parser.parse_args()
    torch.set_num_threads(1)
    model = PPO.load(args.run_dir / 'best/best_model.zip', device='cpu')
    env = WheelzGoalEnv(args.run_dir / 'robot_config.json')
    fig, axes = plt.subplots(3, 3, figsize=(10, 10), constrained_layout=True)
    for index, ax in enumerate(axes.flat):
        seed = 200000 + index
        obs, _ = env.reset(seed=seed)
        trajectory = [env.pose[:2].copy()]
        for step in range(env.max_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            trajectory.append(env.pose[:2].copy())
            if terminated or truncated:
                break
        path = np.asarray(trajectory)
        ax.plot(path[:, 0], path[:, 1], color='#2672b8', lw=2)
        ax.scatter(*path[0], color='#222222', marker='o', label='Start', s=25)
        ax.scatter(*env.goal, color='#2b8d46', marker='*', s=130, label='Goal')
        ax.add_patch(plt.Circle(env.goal, env.goal_tolerance, fill=False, color='#2b8d46'))
        ax.set(xlim=(-2, 2), ylim=(-2, 2), aspect='equal',
               title=f"Seed {seed}: {'settled' if info['is_success'] else 'failed'}, {(step+1)*env.dt:.1f}s",
               xlabel='x (m)', ylabel='y (m)')
        ax.grid(alpha=.2)
    axes[0, 0].legend(loc='upper right', fontsize=8)
    fig.suptitle('Wheelz PPO — first nine held-out simulation episodes\nFive fixed-power actions; randomized dynamics; no obstacles')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)
    env.close()
    print(args.output)


if __name__ == '__main__':
    main()
