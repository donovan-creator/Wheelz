# Wheelz five-action goal policy

Trained locally on 2026-10-02 with PPO (Stable-Baselines3 2.7.1, PyTorch 2.14.0 CPU).
The experiment ran **1,001,472 transitions**; the deployed checkpoint was selected
at **950,000 transitions** using 40 fixed validation episodes. The actor is
9 inputs -> 128 tanh -> 128 tanh -> 5 categorical logits.

## Independent simulation evaluation

200 episode seeds 200000–200199, not used for checkpoint selection, with the same
domain-randomization distribution as training:

| Metric | Result |
| --- | --- |
| Reached and settled at goal | 200 / 200 |
| Observed success rate | 100% |
| Wilson 95% interval | 98.1%–100% |
| Mean final goal distance | 0.0244 m |
| Mean simulated episode duration | 5.671 s |
| Boundary exits / timeouts | 0 / 0 |
| Random-action baseline | 0 / 200 |
| Simple unoptimized steering heuristic | 23 / 200 |

The heuristic is a basic rotate/drive rule, not a tuned classical navigation
controller; this comparison does not establish superiority to classical control.
The policy used stop, forward and both turn actions on these seeds; it never
selected backward, although backward remains one of the five available actions.

Success requires distance <=0.10 m, simulated wheel speeds <0.06 m/s and requesting
stop for three consecutive 0.1 s steps. The full per-episode results are in
`evaluation.json`. The local training checkpoint, configuration and logs are
under `pc/reinforcement_learning/runs/discrete_ppo_v2/`.
The first nine consecutive held-out trajectories are plotted there in
`trajectories.png`.

## Scope and limitations

- Inputs: goal relative to robot, velocity and previous action; no camera input.
- Scene: 4 m x 4 m empty plane, random initial pose and goal.
- Dynamics: estimated 0.45 m/s wheel speed, 0.16 s response time, 166.32 mm track.
- Randomization: wheel strength +/-12%, track +/-6%, response scale 0.75–1.35,
  feature noise std 0.003, and 0–0.1 s command delay.
- No obstacles, terrain, real HTTP timing model, wheel slip model, localization
  faults, battery voltage curve, or physical data validation.
- One training seed (42). The interval describes sampled test episodes, not
  variability across independent training runs or real-world performance.
- No ESP firmware changes. Fixed-power actions only. No onboard Wi-Fi watchdog.
- The ROS node adds an independent stop guard inside goal tolerance. The metrics
  above evaluate the raw learned policy, without that ROS guard.

## Artifacts and checks

`wheelz_discrete_policy.npz` contains float32 actor matrices and JSON metadata,
loaded with `allow_pickle=False`. The metadata includes the full configuration,
action ordering, contract version, training step count and checkpoint SHA-256.
Its logits and discrete actions match PPO on 4096 random observations. An ONNX
copy was also exported and checked, in the local experiment directory.

The active ROS node starts disabled and publishes to `/wheelz/policy_cmd_vel`.
It requires fresh odometry, explicit enable, and a new `goal_pose` in `odom`.
It never enables the hardware bridge itself. Stale/invalid odometry, an invalid
goal, episode timeout or an explicit disable stops and cancels the active goal.
There is no automatic resume after an odometry fault.

The trained actor is **not validated for physical deployment**. Measure dynamics,
calibrate odometry and check motor direction before supervised hardware trials.
