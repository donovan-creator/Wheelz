# Wheelz five-action reinforcement learning

The active trainer uses **PPO with Discrete(5)** to navigate to a 2-D goal and
stop. It generates experience in simulation; no recorded robot dataset is needed.
Cameras, obstacles, maps and collision avoidance are not included.

The previous continuous SAC code was backed up under
`runs/continuous_backup_20261002_120345/`. Existing SAC checkpoints and ONNX files
remain in `runs/`; they are incompatible with this discrete policy. New runs use
`runs/discrete_ppo_v2/`.

## Train and evaluate (PowerShell)

An existing `.venv` on this PC has the dependencies. On another machine, create
a virtual environment and install `requirements.txt`.

```powershell
cd C:\Users\donov\Desktop\stuff\career\Projects\Wheelz\pc\reinforcement_learning
.\.venv\Scripts\python.exe smoke_test.py
.\.venv\Scripts\python.exe train.py --steps 1000000 --run-dir runs/my_experiment
.\.venv\Scripts\python.exe evaluate.py --episodes 200 --baselines
.\.venv\Scripts\python.exe evaluate.py --episodes 5 --watch
.\.venv\Scripts\python.exe export_onnx.py
```

The evaluate/export defaults refer to the delivered `runs/discrete_ppo_v2`
experiment. Supply `--model` and `--config` to evaluate another run.
The trainer refuses to overwrite an existing run unless `--resume` is provided:

```powershell
.\.venv\Scripts\python.exe train.py --steps 300000 --run-dir runs/discrete_ppo_v2 --resume runs/discrete_ppo_v2/wheelz_ppo_final.zip
```

Resume uses that run's saved configuration, not an edited global config.
Each experiment saves its config, source snapshots, validation history, TensorBoard
logs, checkpoints and final PPO model. The best model is selected first by goal
success rate and then by mean return on 40 fixed validation seeds starting at
10000. Held-out evaluation uses seeds starting at 200000. Do not tune on the final
test results and then describe those same seeds as unseen.

## Action and observation contract

| Index | Semantic action | Left/right motor target | Default ESP endpoint via ROS |
| --- | --- | --- | --- |
| 0 | stop | 0, 0 | /stop |
| 1 | forward | +1, +1 | /forward |
| 2 | backward | -1, -1 | /backward |
| 3 | left (positive yaw) | -1, +1 | /right |
| 4 | right (negative yaw) | +1, -1 | /left |

The endpoint turn names are reversed relative to normal differential-drive yaw
in the checked-in firmware. The ROS bridge handles that mapping. Check actual
wiring and adjust `positive_yaw_action` before any physical test.

All actions are fixed-power targets with modeled motor inertia. There is no
proportional-speed command. The action interval is 0.1 seconds. Training randomizes
motor strength, motor response, track width, small observation noise, and 0–1
steps of command delay. These are approximations, not measured hardware dynamics.

The nine float32 features are robot-frame goal x/y divided by twice the world
half extent; sine/cosine of goal bearing; normalized linear/angular velocity;
the previous requested action's two wheel targets; and normalized goal distance.
Features are clipped to [-1.5, 1.5]. Simulation and ROS import the same
`wheelz_ros.policy_contract.observation` function.

Success means being within 0.10 m of the goal, with both simulated wheel speeds
below 0.06 m/s, requesting stop for three consecutive steps. An episode ends
after success, crossing the simulation boundary, or 400 steps (40 seconds).
Dense progress reward guides learning; there is no recurring positive reward
for waiting near the goal. This avoids rewarding an endless near-goal loop.

Wheel diameter is 43 mm and track width is estimated at 166.32 mm from prior
project data. The assumed 0.45 m/s full-power wheel speed and 0.16 s response time
must be measured. The simulated scene has no obstacles.

## Delivered inference formats

`export_onnx.py` now exports the PPO actor to the ROS package's
`models/wheelz_discrete_policy.npz` by default. Despite the historical script
name, NumPy is the primary format: ROS needs no PyTorch installation.
The artifact includes actor weights, action ordering, normalization/configuration,
training step count and source checkpoint SHA-256. It uses no pickled objects.

Export checks 4096 observations against PyTorch, including all discrete decisions.
For an additional ONNX export:

```powershell
.\.venv\Scripts\python.exe export_onnx.py --onnx runs/discrete_ppo_v2/wheelz_policy.onnx
```

The ONNX output is **five action logits**, not wheel speeds. Apply argmax over
the last dimension to select the action. Continuous SAC outputs must not be used.

## ROS use

See [ROS policy operation](../../ros2_ws/README.md#learned-goal-policy).
The node starts disabled and publishes a separate proposal topic. It requires
fresh calibrated odometry and an explicit goal in the odom frame. It neither
starts automatically with the bridge nor enables the bridge's motors.

The simulator result is not hardware validation. There is no camera/obstacle
reasoning, and the unchanged ESP cannot stop itself when Wi-Fi is lost. Collect
ROS bags to calibrate dynamics before supervised physical trials.

## Fresh GitHub checkout

The portable actor and full PPO checkpoint are included in
`ros2_ws/src/wheelz_ros/models/`. Local experiment logs in `runs/` are ignored.
To evaluate a fresh checkout from this directory:

```powershell
.\.venv\Scripts\python.exe evaluate.py --model ../../ros2_ws/src/wheelz_ros/models/wheelz_ppo_checkpoint.zip --config ../../ros2_ws/src/wheelz_ros/models/robot_config.json --episodes 5 --watch
```
