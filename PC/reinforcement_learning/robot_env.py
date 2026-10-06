"""Five-action goal-navigation environment matching Wheelz's fixed-power interface."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

ROS_SOURCE = Path(__file__).resolve().parents[2] / 'ros2_ws' / 'src' / 'wheelz_ros'
if str(ROS_SOURCE) not in sys.path:
    sys.path.insert(0, str(ROS_SOURCE))
from wheelz_ros.policy_contract import ACTION_NAMES, WHEEL_TARGETS, observation

CONFIG_PATH = Path(__file__).with_name('robot_config.json')


def load_config(path: str | Path = CONFIG_PATH) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding='utf-8'))


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class WheelzGoalEnv(gym.Env):
    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 10}

    def __init__(self, config_path=CONFIG_PATH, render_mode=None, domain_randomization=True):
        super().__init__()
        self.cfg = load_config(config_path)
        self.render_mode, self.domain_randomization = render_mode, domain_randomization
        self.dt = float(self.cfg['simulation']['control_dt_s'])
        self.world = float(self.cfg['task']['world_half_extent_m'])
        self.max_steps = int(self.cfg['task']['max_episode_steps'])
        self.goal_tolerance = float(self.cfg['task']['goal_tolerance_m'])
        self.nominal_max_speed = float(self.cfg['robot']['max_wheel_speed_mps'])
        self.nominal_track = float(self.cfg['robot']['track_width_m'])
        self.action_space = spaces.Discrete(len(ACTION_NAMES))
        self.observation_space = spaces.Box(-1.5, 1.5, (9,), np.float32)
        self.pose = np.zeros(3)
        self.goal = np.zeros(2)
        self.wheel_speed = np.zeros(2)
        self.last_action = 0
        self.steps = self.settled = 0
        self._screen = self._clock = None

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.pose[:2] = self.np_random.uniform(-self.world + .25, self.world - .25, 2)
        self.pose[2] = self.np_random.uniform(-math.pi, math.pi)
        for _ in range(100):
            self.goal = self.np_random.uniform(-self.world + .25, self.world - .25, 2)
            if self._distance() >= self.cfg['task']['minimum_goal_distance_m']:
                break
        self.wheel_speed[:] = 0
        self.last_action = self.steps = self.settled = 0
        self._sample_dynamics()
        self.pending = [0] * self.delay_steps
        self._previous_distance = self._distance()
        if self.render_mode == 'human':
            self.render()
        return self._observation(), self._info(False, False)

    def _sample_dynamics(self):
        r = self.cfg['domain_randomization']
        if self.domain_randomization:
            scale = lambda key: self.np_random.uniform(*r[key])
            self.track_width = self.nominal_track * scale('track_width_scale')
            self.max_left = self.nominal_max_speed * scale('motor_strength_scale')
            self.max_right = self.nominal_max_speed * scale('motor_strength_scale')
            self.motor_tau = self.cfg['robot']['motor_time_constant_s'] * scale('motor_tau_scale')
            self.obs_noise = r['observation_noise_std']
            self.delay_steps = int(self.np_random.integers(0, r['max_action_delay_steps'] + 1))
        else:
            self.track_width = self.nominal_track
            self.max_left = self.max_right = self.nominal_max_speed
            self.motor_tau = self.cfg['robot']['motor_time_constant_s']
            self.obs_noise = 0.0
            self.delay_steps = 0

    def step(self, action):
        if not self.action_space.contains(action):
            raise ValueError(f'Expected action index 0..4, got {action!r}')
        action = int(action)
        self.pending.append(action)
        applied = self.pending.pop(0)
        target = np.asarray(WHEEL_TARGETS[applied]) * (self.max_left, self.max_right)
        # Substeps keep fast full-power turns and inertia numerically stable.
        h = self.dt / 5
        for _ in range(5):
            self.wheel_speed += (1 - math.exp(-h / self.motor_tau)) * (target - self.wheel_speed)
            left, right = self.wheel_speed
            linear = .5 * (left + right)
            angular = (right - left) / self.track_width
            mid = self.pose[2] + .5 * angular * h
            self.pose[0] += linear * math.cos(mid) * h
            self.pose[1] += linear * math.sin(mid) * h
            self.pose[2] = wrap_angle(self.pose[2] + angular * h)
        self.steps += 1
        distance = self._distance()
        inside = distance <= self.goal_tolerance
        stationary = float(np.max(np.abs(self.wheel_speed))) < self.cfg['task']['settled_speed_mps']
        self.settled = self.settled + 1 if inside and stationary and action == 0 else 0
        success = self.settled >= self.cfg['task']['settle_steps']
        out = bool(np.any(np.abs(self.pose[:2]) > self.world))
        rc = self.cfg['reward']
        reward = rc['progress'] * (self._previous_distance - distance) - rc['time']
        reward -= rc['switch'] * int(action != self.last_action)
        reward -= rc['effort'] * float(np.square(WHEEL_TARGETS[action]).mean())
        if inside:
            reward += rc['near_goal_stop'] if action == 0 else -rc['near_goal_motion']
        if success:
            reward += rc['success']
        elif out:
            reward -= rc['out_of_bounds']
        self.last_action = action
        self._previous_distance = distance
        if self.render_mode == 'human':
            self.render()
        return (self._observation(), float(reward), bool(success or out),
                self.steps >= self.max_steps, self._info(success, out))

    def _distance(self):
        return float(np.linalg.norm(self.goal - self.pose[:2]))

    def _observation(self):
        left, right = self.wheel_speed
        obs = np.asarray(observation(
            *self.pose, *self.goal, .5 * (left + right),
            (right - left) / self.track_width, self.last_action, self.cfg), dtype=np.float32)
        if self.obs_noise:
            obs += self.np_random.normal(0, self.obs_noise, obs.shape).astype(np.float32)
        return np.clip(obs, -1.5, 1.5).astype(np.float32)

    def _info(self, success, out):
        return {'is_success': success, 'out_of_bounds': out, 'distance_to_goal': self._distance(),
                'pose': self.pose.copy(), 'goal': self.goal.copy(),
                'action': ACTION_NAMES[self.last_action], 'settled_steps': self.settled}

    def render(self):
        import pygame
        size = 720
        if self._screen is None:
            pygame.init()
            self._screen = (pygame.display.set_mode((size, size))
                            if self.render_mode == 'human' else pygame.Surface((size, size)))
            pygame.display.set_caption('Wheelz five-action RL simulator')
            self._clock = pygame.time.Clock()
        screen = self._screen
        screen.fill((245, 247, 250))
        scale = size / (2 * self.world)
        def px(point):
            return int((point[0] + self.world) * scale), int((self.world - point[1]) * scale)
        for value in np.arange(-self.world, self.world + .01, .5):
            pygame.draw.line(screen, (220, 224, 230), px((value, -self.world)), px((value, self.world)))
            pygame.draw.line(screen, (220, 224, 230), px((-self.world, value)), px((self.world, value)))
        pygame.draw.circle(screen, (46, 170, 90), px(self.goal), max(5, int(self.goal_tolerance * scale)), 3)
        center = np.array(px(self.pose[:2]))
        radius = max(9, int(self.cfg['robot']['body_radius_m'] * scale))
        pygame.draw.circle(screen, (40, 105, 210), center, radius)
        tip = center + np.array([math.cos(self.pose[2]), -math.sin(self.pose[2])]) * radius * 1.4
        pygame.draw.line(screen, (255, 255, 255), center, tip, 4)
        if self.render_mode == 'human':
            pygame.event.pump()
            pygame.display.flip()
            self._clock.tick(round(1 / self.dt))
            return None
        return np.transpose(pygame.surfarray.array3d(screen), (1, 0, 2))

    def close(self):
        if self._screen is not None:
            import pygame
            pygame.quit()
            self._screen = None


RobotEnv = WheelzGoalEnv
