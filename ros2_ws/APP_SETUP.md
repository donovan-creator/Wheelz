# Use Toolbox's Automatic tab

The phone sends coordinates to an authenticated HTTP API in WSL. The PC runs the
included trained PPO model and the ROS motor bridge. No model runs on the phone;
no ESP firmware update is needed.

## Start the host

In Ubuntu/WSL, from this repository's `ros2_ws` directory:

```bash
bash scripts/build.sh
source install/setup.bash
# Demo only: all motors/sensors below are simulated.
ros2 launch wheelz_ros app.launch.py mock:=true api_host:=0.0.0.0
```

In a second Ubuntu terminal, display the generated pairing token:

```bash
cat ~/.config/wheelz/app-token
```

The token is generated locally, not committed to Git. Keep it on your trusted
robot network. This API is plain HTTP and must not be exposed to the Internet.
Restart the host after changing/replacing its token file.

For the physical robot, use the calibrated configuration described in
[README.md](README.md#calibrate-odometry):

```bash
ros2 launch wheelz_ros app.launch.py mock:=false \
  robot_url:=http://ESP8266_IP config:="$PWD/config.local.yaml" api_host:=0.0.0.0
```

Do not launch the separate `policy` executable at the same time: the app gateway
already contains that node. The app launch routes its commands to the motor
bridge. Motors remain disabled until Navigate or a held manual direction is
explicitly requested through the app.

## Reach WSL from your phone

Both phone and PC must be on the robot's reachable network. A browser running on
this PC can reach `http://127.0.0.1:8766`; that is **not** the address to enter on
the phone.

With default WSL NAT, the phone needs a Windows port forward. In **Administrator
PowerShell**, use a separate Windows port (8767) to avoid localhost-relay conflicts:

```powershell
$wheelzWslAddress = (wsl -d Ubuntu -- hostname -I).Trim().Split(' ', [StringSplitOptions]::RemoveEmptyEntries)[0]
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8767 connectaddress=$wheelzWslAddress connectport=8766
New-NetFirewallRule -Name WheelzAppAPI -DisplayName 'Wheelz app API' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8767 -RemoteAddress LocalSubnet -Profile Private
ipconfig
```

Enter `http://PC_WIFI_IPV4:8767` in the app, using the PC's Wi-Fi/Ethernet IPv4
from `ipconfig`. The PC network must be marked Private for that firewall rule.
Wi-Fi client isolation or phone hotspots can prevent peer connections even when
both devices show the same network.

The WSL address can change after a restart. Update the existing forward:

```powershell
$wheelzWslAddress = (wsl -d Ubuntu -- hostname -I).Trim().Split(' ', [StringSplitOptions]::RemoveEmptyEntries)[0]
netsh interface portproxy set v4tov4 listenaddress=0.0.0.0 listenport=8767 connectaddress=$wheelzWslAddress connectport=8766
```

These network changes are optional setup instructions; development/testing did
not change your firewall, port forwarding, or WSL networking configuration.
If you already use mirrored networking, direct access to the PC's port 8766 may
work with the appropriate Windows/Hyper-V firewall rule instead.

To remove the NAT forwarding later (Administrator PowerShell):

```powershell
netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=8767
Remove-NetFirewallRule -Name WheelzAppAPI
```

## In the app

1. Open **Automatic**. Enter the PC address and pairing token, then **Connect to PC**.
2. Check position/readiness. Physical navigation stays unavailable until odometry
   is calibrated and current.
3. Enter X/Y in metres and press **Navigate**. These are absolute coordinates in
   the odometry frame established when the driver started: initial forward is
   +X, initial left is +Y. They are not relative offsets from the current pose.
4. Watch navigation status, or press **CANCEL / STOP**.
5. Switching to **Manual** cancels navigation and disables the previous command.
   While paired, manual controls also use ROS; hold a direction and release to stop.

**Stop and disconnect** requires acknowledgment from the ROS bridge before the
app returns to direct ESP control. Do not run the ROS host while using direct
ESP mode: the bridge's repeated stop commands can conflict with direct movement.
The direct mode's sensor display, gyro calibration, run IDs and manual cloud
logging are retained. Cloud replies never control motors.

The app holds connection settings/token only for its current session. Reopening
requires pairing again. Backgrounding the app requests stop. Navigation needs a
heartbeat every 3 seconds; manual commands expire after 0.6 seconds. A stale
command sequence cannot override a newer stop. These are host protections:
if the ESP itself loses Wi-Fi, it still has **no onboard watchdog**.

Goals are limited to 4 m from the current odometry position. There is no obstacle
avoidance. Simulation success is not physical calibration or hardware validation.

## API

All endpoints require `Authorization: Bearer TOKEN`. Status is read-only.
Commands are JSON POSTs with a nonempty `client_id` and increasing integer
`sequence` per client session:

| Method/path | Body | Behavior |
| --- | --- | --- |
| GET /status | — | Readiness, position, goal and navigation state |
| POST /navigate | x, y, client_id, sequence | Validate goal, enable bridge, start learned policy |
| POST /manual | action, client_id, sequence | Cancel policy; hold a semantic action |
| POST /stop | client_id, sequence | Cancel policy and disable bridge; best-effort physical stop |
| POST /heartbeat | client_id | Maintain the current app session's navigation lease |

Manual action strings: stop, forward, backward, left, right. Left/right mean
standard ROS yaw; the bridge resolves the firmware endpoint naming.
Successful stop responses acknowledge the host/bridge request, not measured
physical standstill. Stop is allowed from another authenticated session so a
second operator can cancel motion.

For Flutter web development, permit only your exact test origin, for example
`allowed_origins:=http://127.0.0.1:7357`. Native mobile apps do not require CORS.
The test suite exercises the real HTTP API against ROS and the ESP emulator.
