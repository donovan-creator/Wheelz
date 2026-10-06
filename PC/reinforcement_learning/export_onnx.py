"""Export the discrete actor for ROS NumPy inference and optional ONNX inference."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO

from robot_env import ROS_SOURCE, load_config
from wheelz_ros.policy_contract import ACTION_NAMES, CONTRACT_VERSION
from wheelz_ros.policy_runtime import DiscretePolicy

HERE = Path(__file__).resolve().parent


class ActorLogits(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.actor = policy.mlp_extractor.policy_net
        self.output = policy.action_net

    def forward(self, obs):
        return self.output(self.actor(obs))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=HERE / 'runs/discrete_ppo_v2/best/best_model.zip')
    parser.add_argument('--config', type=Path, default=HERE / 'runs/discrete_ppo_v2/robot_config.json')
    parser.add_argument('--output', type=Path,
                        default=ROS_SOURCE / 'models' / 'wheelz_discrete_policy.npz')
    parser.add_argument('--onnx', type=Path, help='Optional additional ONNX artifact')
    args = parser.parse_args()
    torch.set_num_threads(1)
    model = PPO.load(args.model, device='cpu')
    if model.action_space.n != 5 or model.observation_space.shape != (9,):
        raise ValueError('Expected five-action/nine-feature PPO')
    actor = ActorLogits(model.policy).eval()
    layers = [layer for layer in actor.actor if isinstance(layer, torch.nn.Linear)] + [actor.output]
    if len(layers) != 3 or any(not isinstance(layer, (torch.nn.Linear, torch.nn.Tanh))
                               for layer in actor.actor):
        raise ValueError('Portable export supports the configured two-layer tanh actor only')
    metadata = {'contract': CONTRACT_VERSION, 'actions': list(ACTION_NAMES),
                'config': load_config(args.config), 'algorithm': 'PPO',
                'training_timesteps': model.num_timesteps,
                'checkpoint_sha256': hashlib.sha256(args.model.read_bytes()).hexdigest(),
                'hardware_validated': False}
    arrays = {'metadata': np.asarray(json.dumps(metadata))}
    for index, layer in enumerate(layers):
        arrays[f'w{index}'] = layer.weight.detach().numpy().T.copy()
        arrays[f'b{index}'] = layer.bias.detach().numpy().copy()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **arrays)
    portable = DiscretePolicy(args.output)
    samples = np.random.default_rng(710).uniform(-1.5, 1.5, (4096, 9)).astype(np.float32)
    with torch.no_grad():
        expected = actor(torch.from_numpy(samples)).numpy()
    np.testing.assert_allclose(portable.logits(samples), expected, rtol=2e-4, atol=1e-5)
    np.testing.assert_array_equal(portable.predict(samples),
                                  model.predict(samples, deterministic=True)[0])
    if args.onnx:
        args.onnx.parent.mkdir(parents=True, exist_ok=True)
        torch.onnx.export(actor, torch.zeros((1, 9)), args.onnx,
                          input_names=['observation'], output_names=['action_logits'],
                          dynamic_axes={'observation': {0: 'batch'}, 'action_logits': {0: 'batch'}},
                          opset_version=17, dynamo=False)
        import onnxruntime as ort
        actual = ort.InferenceSession(str(args.onnx)).run(None, {'observation': samples})[0]
        np.testing.assert_allclose(actual, expected, rtol=2e-4, atol=1e-5)
        np.testing.assert_array_equal(actual.argmax(-1), expected.argmax(-1))
    print(f'Exported {args.output}; verified logits and all 4096 discrete decisions against PPO.')


if __name__ == '__main__':
    main()
