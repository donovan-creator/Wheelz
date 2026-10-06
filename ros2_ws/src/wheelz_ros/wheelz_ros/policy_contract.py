"""Shared simulation/inference contract. Left/right mean ROS yaw, not URL names."""
import math

CONTRACT_VERSION = 'wheelz-discrete-v1'
ACTION_NAMES = ('stop', 'forward', 'backward', 'left', 'right')
WHEEL_TARGETS = ((0.0, 0.0), (1.0, 1.0), (-1.0, -1.0),
                 (-1.0, 1.0), (1.0, -1.0))
TWIST_TARGETS = ((0.0, 0.0), (0.2, 0.0), (-0.2, 0.0),
                 (0.0, 0.5), (0.0, -0.5))


def observation(x, y, yaw, goal_x, goal_y, linear, angular, previous_action, config):
    """Nine bounded features, identical on the ROS host and in the simulator."""
    world = float(config['task']['world_half_extent_m'])
    speed = float(config['robot']['max_wheel_speed_mps'])
    track = float(config['robot']['track_width_m'])
    dx, dy = goal_x - x, goal_y - y
    c, s = math.cos(yaw), math.sin(yaw)
    gx, gy = c * dx + s * dy, -s * dx + c * dy
    bearing = math.atan2(gy, gx)
    values = [gx / (2 * world), gy / (2 * world),
              math.sin(bearing), math.cos(bearing), linear / speed,
              angular / (2 * speed / track), *WHEEL_TARGETS[previous_action],
              math.hypot(dx, dy) / (2 * world)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Nonfinite policy observation')
    return [max(-1.5, min(1.5, value)) for value in values]
