"""Train a categorical PPO policy; existing SAC runs are retained separately."""
import argparse
import json
from pathlib import Path
import shutil

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env

from evaluate import evaluate
from robot_env import CONFIG_PATH, WheelzGoalEnv

HERE = Path(__file__).resolve().parent


class GoalEvaluation(BaseCallback):
    """Select checkpoints by completed goals, not shaped reward alone."""
    def __init__(self, run_dir, config):
        super().__init__()
        self.run_dir, self.config = run_dir, config
        self.best = (-1.0, -float('inf'))
        self.next_evaluation = 0

    def _on_training_start(self):
        self.next_evaluation = self.num_timesteps + 50_000
        result_path = self.run_dir / 'best' / 'validation.json'
        if result_path.exists():
            result = json.loads(result_path.read_text())
            self.best = (result['success_rate'], result['mean_return'])

    def _on_step(self):
        if self.num_timesteps < self.next_evaluation:
            return True
        self.next_evaluation += 50_000
        result = evaluate(lambda obs, cfg: self.model.predict(obs, deterministic=True)[0],
                          self.config, 40, 10_000, randomized=True)
        print(f"Validation step={self.num_timesteps}: "
              f"{result['successes']}/40 settled goals; "
              f"return={result['mean_return']:.2f}", flush=True)
        self.logger.record('eval/success_rate', result['success_rate'])
        self.logger.record('eval/mean_reward', result['mean_return'])
        result['timesteps'] = self.num_timesteps
        with (self.run_dir / 'validation_history.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({k: v for k, v in result.items() if k != 'records'}) + '\n')
        score = (result['success_rate'], result['mean_return'])
        if score > self.best:
            self.best = score
            target = self.run_dir / 'best'
            target.mkdir(exist_ok=True)
            self.model.save(target / 'best_model')
            (target / 'validation.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--steps', type=int, default=1_000_000)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--run-dir', type=Path, default=HERE / 'runs' / 'discrete_ppo_v2')
    parser.add_argument('--config', type=Path, default=CONFIG_PATH)
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error('--steps must be positive')
    if args.run_dir.exists() and any(args.run_dir.iterdir()) and not args.resume:
        parser.error('Use a new --run-dir or --resume; existing experiments are not overwritten')
    args.run_dir.mkdir(parents=True, exist_ok=True)
    snapshot = args.run_dir / 'robot_config.json'
    if args.resume:
        if not snapshot.exists():
            parser.error('Resume requires the original run directory with robot_config.json')
    else:
        shutil.copyfile(args.config, snapshot)
        for name in ('robot_env.py', 'train.py', 'evaluate.py'):
            shutil.copyfile(HERE / name, args.run_dir / name)
    torch.set_num_threads(1)
    env = make_vec_env(WheelzGoalEnv, n_envs=8, seed=args.seed,
                       env_kwargs={'config_path': snapshot})
    model = (PPO.load(args.resume, env=env, device='cpu') if args.resume else
             PPO('MlpPolicy', env, learning_rate=3e-4, n_steps=256,
                 batch_size=256, n_epochs=10, gamma=.995, gae_lambda=.95,
                 ent_coef=.01, policy_kwargs={'net_arch': [128, 128]},
                 tensorboard_log=str(args.run_dir / 'tensorboard'),
                 seed=args.seed, device='cpu', verbose=1))
    callbacks = [
        CheckpointCallback(save_freq=12_500, save_path=str(args.run_dir / 'checkpoints')),
        GoalEvaluation(args.run_dir, snapshot),
    ]
    try:
        model.learn(total_timesteps=args.steps, callback=callbacks,
                    reset_num_timesteps=not bool(args.resume), log_interval=10)
        model.save(args.run_dir / 'wheelz_ppo_final')
        (args.run_dir / 'training_summary.json').write_text(json.dumps({
            'algorithm': 'PPO', 'action_space': 'Discrete(5)', 'seed': args.seed,
            'num_timesteps': model.num_timesteps, 'device': 'cpu',
            'torch_version': torch.__version__,
        }, indent=2), encoding='utf-8')
    finally:
        env.close()


if __name__ == '__main__':
    main()
