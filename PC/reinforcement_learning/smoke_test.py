"""Check API, five-action dynamics, settled success and shared ROS observations."""
import math

from gymnasium.utils.env_checker import check_env
import numpy as np

from robot_env import WheelzGoalEnv
from wheelz_ros.policy_contract import observation


def main():
    env = WheelzGoalEnv(domain_randomization=False)
    check_env(env, skip_render_check=True)
    for action, forward_sign, yaw_sign in ((1, 1, 0), (2, -1, 0), (3, 0, 1), (4, 0, -1)):
        env.reset(seed=42)
        env.pose[:] = 0
        env.goal[:] = (1, 1)
        env._previous_distance = env._distance()
        for _ in range(3):
            env.step(action)
        if forward_sign:
            assert env.pose[0] * forward_sign > 0
        if yaw_sign:
            assert env.pose[2] * yaw_sign > 0
        left, right = env.wheel_speed
        expected = observation(*env.pose, *env.goal, .5 * (left + right),
                               (right - left) / env.track_width, action, env.cfg)
        np.testing.assert_allclose(env._observation(), expected, atol=1e-7)
    env.reset(seed=7)
    env.goal[:] = env.pose[:2]
    env._previous_distance = 0
    for step in range(env.cfg['task']['settle_steps']):
        _, _, terminated, _, info = env.step(0)
        assert terminated == (step == env.cfg['task']['settle_steps'] - 1)
    assert info['is_success']
    for invalid in (5, -1, np.array([.1, .2])):
        try:
            env.step(invalid)
        except ValueError:
            continue
        raise AssertionError('Accepted invalid action')
    # Randomized resets with identical seeds reproduce delay, noise and physics.
    first, second = WheelzGoalEnv(), WheelzGoalEnv()
    a, _ = first.reset(seed=31)
    b, _ = second.reset(seed=31)
    np.testing.assert_array_equal(a, b)
    for action in (1, 3, 0, 2, 4):
        np.testing.assert_array_equal(first.step(action)[0], second.step(action)[0])
    env.close()
    first.close()
    second.close()
    print('Environment API, five actions, settled success, deterministic seeds and ROS observation parity passed.')


if __name__ == '__main__':
    main()
