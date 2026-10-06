"""Portable PPO actor inference; no PyTorch, pickle, or ONNX runtime needed."""
import json
from pathlib import Path

import numpy as np

from .policy_contract import ACTION_NAMES, CONTRACT_VERSION


class DiscretePolicy:
    def __init__(self, path):
        self.path = Path(path)
        with np.load(self.path, allow_pickle=False) as archive:
            self.metadata = json.loads(str(archive['metadata'].item()))
            self.layers = [(archive[f'w{i}'].copy(), archive[f'b{i}'].copy())
                           for i in range(3)]
        if self.metadata['contract'] != CONTRACT_VERSION:
            raise ValueError('Incompatible policy contract')
        if self.metadata['actions'] != list(ACTION_NAMES):
            raise ValueError('Incompatible action ordering')
        expected = [(9, 128), (128, 128), (128, 5)]
        for (weight, bias), shape in zip(self.layers, expected):
            if weight.shape != shape or bias.shape != (shape[1],):
                raise ValueError('Unexpected actor shape')
            if not np.isfinite(weight).all() or not np.isfinite(bias).all():
                raise ValueError('Nonfinite policy weights')
        self.config = self.metadata['config']

    def logits(self, obs):
        values = np.asarray(obs, dtype=np.float32)
        if values.shape[-1:] != (9,) or not np.isfinite(values).all():
            raise ValueError('Expected finite observations with nine features')
        for index, (weight, bias) in enumerate(self.layers):
            values = values @ weight + bias
            if index < 2:
                values = np.tanh(values)
        return values

    def predict(self, obs):
        return np.argmax(self.logits(obs), axis=-1)
