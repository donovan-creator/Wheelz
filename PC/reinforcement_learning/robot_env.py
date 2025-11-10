import requests
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces

class RobotEnv(gym.Env):
    """
    Custom Gym environment for ESP8266 Robot.
    Actions are mapped to HTTP endpoints, state is IMU + encoder data.
    """

    def __init__(self, base_url="http://172.20.10.3"):
        super().__init__()
        self.base_url = base_url
        self.last_left = 0
        self.last_right = 0
        self.last_time = time.time()

        # Define actions
        # 0=stop, 1=forward, 2=left, 3=right, 4=backward
        self.action_space = spaces.Discrete(5)

        # Define observation space: [ax, ay, az, gx, gy, gz, d_left, d_right]
        high = np.array([np.inf]*8, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    def step(self, action):
        """
        Execute one action on the real robot, then observe state & reward.
        """
        endpoint = {0:'stop', 1:'forward', 2:'left', 3:'right', 4:'backward'}[action]

        # --- Send the command ---
        try:
            requests.get(f"{self.base_url}/{endpoint}", timeout=0.5)
        except Exception as e:
            print("Command error:", e)

        time.sleep(0.5)  # short motion duration

        # --- Read sensors ---
        imu = self._read_json("/imu")
        counts = self._read_text("/counts")

        state = self._compose_state(imu, counts)
        reward = self._compute_reward(state)
        done = False  # RL episode termination condition (can add later)
        info = {}

        return np.array(state, dtype=np.float32), reward, done, info

    def reset(self, seed=None, options=None):
        """Reset robot to neutral state."""
        super().reset(seed=seed)
        self.last_left = 0
        self.last_right = 0
        self.last_time = time.time()
        self._send_cmd("stop")
        time.sleep(1.0)
        state = self._compose_state(self._read_json("/imu"), self._read_text("/counts"))
        return np.array(state, dtype=np.float32), {}

    # --- Helpers ---
    def _send_cmd(self, cmd):
        try:
            requests.get(f"{self.base_url}/{cmd}", timeout=0.5)
        except:
            pass

    def _read_json(self, path):
        try:
            r = requests.get(f"{self.base_url}{path}", timeout=0.5)
            return r.json()
        except:
            return {"ax":0,"ay":0,"az":0,"gx":0,"gy":0,"gz":0}

    def _read_text(self, path):
        try:
            r = requests.get(f"{self.base_url}{path}", timeout=0.5)
            return r.text
        except:
            return "Left:0 | Right:0"

    def _compose_state(self, imu, counts_text):
        # Parse encoder text
        import re
        m = re.search(r"Left:\s*(-?\d+).*Right:\s*(-?\d+)", counts_text)
        left, right = (int(m.group(1)), int(m.group(2))) if m else (0, 0)

        # Delta ticks since last step
        dl = left - self.last_left
        dr = right - self.last_right
        self.last_left, self.last_right = left, right

        # Compose full state
        state = [
            imu.get("ax",0), imu.get("ay",0), imu.get("az",0),
            imu.get("gx",0), imu.get("gy",0), imu.get("gz",0),
            dl, dr
        ]
        return state

    def _compute_reward(self, state):
        # Simple example: reward forward motion (based on encoder delta)
        dl, dr = state[-2], state[-1]
        forward = (dl + dr) / 2.0
        turn = abs(dl - dr)
        return forward - 0.1 * turn
