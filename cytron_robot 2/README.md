# Cytron Robot — Plug-and-Play ROS2 Package

A complete ROS2 package for a differential drive robot using:
- **Cytron MDD3A / MDD10A** motor driver
- **RPLidar A1** lidar
- **Raspberry Pi** (any model with 40-pin GPIO)
- **SLAM Toolbox** for mapping
- **Nav2** for autonomous navigation

---

## Hardware Wiring

### Cytron Motor Driver → Raspberry Pi GPIO

```
╔══════════════════════════════════════════════════════════════╗
║  CYTRON MDD3A / MDD10A  ←→  RASPBERRY PI (BCM numbering)   ║
╠══════════════════════════╦═══════════════════════════════════╣
║  LEFT MOTOR (Channel A)  ║  RIGHT MOTOR (Channel B)          ║
╠══════════════════════════╬═══════════════════════════════════╣
║  PWM1  →  GPIO 12        ║  PWM2  →  GPIO 13                 ║
║          (Phys pin 32)   ║          (Phys pin 33)            ║
║  DIR1  →  GPIO 24        ║  DIR2  →  GPIO 25                 ║
║          (Phys pin 18)   ║          (Phys pin 22)            ║
╠══════════════════════════╩═══════════════════════════════════╣
║  GND   →  Any RPi GND  (Phys pins: 6, 9, 14, 20, 25, 30…)  ║
║  VCC   →  5V RPi pin   (Phys pin 2 or 4)  OR external 5V    ║
║  VMOT  →  Battery+  (7.4V LiPo or 9–12V DC supply)          ║
╚══════════════════════════════════════════════════════════════╝
```

### Raspberry Pi 40-Pin Header Reference

```
         3V3  [ 1][ 2]  5V
    GPIO  2   [ 3][ 4]  5V
    GPIO  3   [ 5][ 6]  GND  ← GND to Cytron
    GPIO  4   [ 7][ 8]  GPIO 14
         GND  [ 9][10]  GPIO 15
    GPIO 17   [11][12]  GPIO 18
    GPIO 27   [13][14]  GND
    GPIO 22   [15][16]  GPIO 23
         3V3  [17][18]  GPIO 24  ← DIR1 (Left motor direction)
    GPIO 10   [19][20]  GND
    GPIO  9   [21][22]  GPIO 25  ← DIR2 (Right motor direction)
    GPIO 11   [23][24]  GPIO  8
         GND  [25][26]  GPIO  7
    GPIO  0   [27][28]  GPIO  1
    GPIO  5   [29][30]  GND
    GPIO  6   [31][32]  GPIO 12  ← PWM1 (Left motor speed)
    GPIO 13   [33][34]  GND
              [33] = GPIO 13 ← PWM2 (Right motor speed)
    GPIO 19   [35][36]  GPIO 16
    GPIO 26   [37][38]  GPIO 20
         GND  [39][40]  GPIO 21
```

### Optional Wheel Encoders → RPi GPIO

```
╔══════════════════════════════════════════════════════╗
║  ENCODER         →  GPIO (BCM)  →  Physical Pin      ║
╠══════════════════════════════════════════════════════╣
║  Left  Channel A →  GPIO 17    →  Pin 11             ║
║  Left  Channel B →  GPIO 27    →  Pin 13             ║
║  Right Channel A →  GPIO 22    →  Pin 15             ║
║  Right Channel B →  GPIO 23    →  Pin 16             ║
║  Encoder VCC     →  3.3V       →  Pin 1 or 17        ║
║  Encoder GND     →  GND        →  Pin 6              ║
╚══════════════════════════════════════════════════════╝
```
> If you have no encoders, set `use_encoders: false` in both launch files.
> The robot will use cmd_vel dead-reckoning (less accurate but functional).

### RPLidar A1 → Raspberry Pi

```
RPLidar USB cable → any RPi USB port
Will appear as /dev/ttyUSB0 (or /dev/rplidar after install script)
```

---

## Installation (One Time)

```bash
# 1. Clone into your workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
# (copy/clone the cytron_robot package here)

# 2. Source ROS2
source /opt/ros/humble/setup.bash

# 3. Run the install script
cd ~/robot_ws/src/cytron_robot
chmod +x scripts/install.sh
./scripts/install.sh

# 4. Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash"    >> ~/.bashrc
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc

# 5. Reboot (required for GPIO/serial permissions)
sudo reboot
```

---

## Tune Robot Dimensions

Edit these values in **both** launch files and the URDF to match your actual robot:

| Parameter | Default | Where to change |
|---|---|---|
| `wheel_radius` | 0.033 m | `motor_driver.py` line, `odometry_node.py`, URDF |
| `wheel_baseline` | 0.160 m | Same 3 places |
| `ticks_per_rev` | 20 | `odometry_node.py` |
| `invert_left` | False | `motor_driver.py` / launch files |
| `invert_right` | True | `motor_driver.py` / launch files |

**Measure wheel_baseline:** Put a ruler from the centre of the left wheel tread to the centre of the right wheel tread.

**Test motor direction:** Run teleop, press `i` to go forward. If a motor spins backwards, set its `invert_*` to True.

---

## Phase 1 — Build the Map

```bash
# Terminal 1: Start mapping
ros2 launch cytron_robot slam_mapping.launch.py

# Terminal 2: Drive the robot through both rooms
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Controls: i=forward  ,=backward  j=left  l=right  k=stop

# Terminal 3: When you've covered both rooms, save the map
ros2 run cytron_robot save_map
```

Map files will be saved to:
```
~/robot_ws/install/cytron_robot/share/cytron_robot/maps/room_map.pgm
~/robot_ws/install/cytron_robot/share/cytron_robot/maps/room_map.yaml
```

---

## Phase 2 — Set Room Coordinates

After saving the map, you need to tell the navigator where Room A and Room B are.

1. On a machine with a display (can be the RPi via VNC or a laptop with ROS2):
   ```bash
   # Launch navigation
   ros2 launch cytron_robot navigation.launch.py

   # Open RViz on your laptop (point to RPi's ROS)
   export ROS_DOMAIN_ID=0
   rviz2
   ```

2. In RViz:
   - Add a **Map** display → Topic: `/map`
   - Add a **LaserScan** display → Topic: `/scan`
   - Use **2D Pose Estimate** tool → click the centre of **Room A** → note x, y in terminal
   - Use **2D Pose Estimate** tool → click the centre of **Room B** → note x, y

3. Edit `cytron_robot/room_navigator.py`:
   ```python
   ROOM_A = {'name': 'Room A', 'x': 1.0, 'y': 0.5, 'z': 0.0, 'w': 1.0}
   ROOM_B = {'name': 'Room B', 'x': 4.5, 'y': 2.0, 'z': 0.0, 'w': 1.0}
   ```

4. Rebuild:
   ```bash
   colcon build --packages-select cytron_robot --symlink-install
   ```

---

## Phase 3 — Navigate Between Rooms

```bash
# Launch everything (motors + lidar + Nav2 + keyboard controller)
ros2 launch cytron_robot navigation.launch.py

# In RViz: click "2D Pose Estimate" and click where the robot is on the map
# (this tells AMCL the starting position)

# In the room_navigator window:
# Press A  →  Robot goes to Room B
# Press S  →  Robot goes to Room A
# Press C  →  Cancel navigation
# Press Q  →  Quit
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| `No module named RPi.GPIO` | Library not installed | `pip3 install RPi.GPIO` |
| `/dev/ttyUSB0: Permission denied` | Serial group | `sudo usermod -aG dialout $USER` then reboot |
| Motors don't move | Wrong pins or direction | Check wiring, set `invert_*` in launch file |
| Robot drives backwards | Motor polarity flipped | Swap motor wires on Cytron terminal, or set `invert_left/right: true` |
| Map is distorted | Odometry drifts without encoders | Add wheel encoders, or drive very slowly |
| AMCL particles scatter | Bad initial pose | Click "2D Pose Estimate" in RViz at robot's actual location |
| Goal rejected | Nav2 not ready | Wait 10–15 seconds after launch for Nav2 to fully start |
| Robot hits walls | Inflation radius too small | Increase `inflation_radius` in `nav2_params.yaml` |

---

## Package Structure

```
cytron_robot/
├── cytron_robot/
│   ├── motor_driver.py      GPIO PWM motor control
│   ├── odometry_node.py     Wheel encoder / dead-reckoning odometry
│   ├── room_navigator.py    Keyboard A/S navigation controller
│   └── save_map.py          Map save utility
├── launch/
│   ├── slam_mapping.launch.py   Phase 1: build map
│   └── navigation.launch.py    Phase 2: navigate
├── config/
│   ├── slam_params.yaml         SLAM Toolbox config
│   └── nav2_params.yaml         Nav2 full config (AMCL, planners, costmaps)
├── urdf/
│   └── robot.urdf               Robot TF description
├── maps/                        Saved map files go here
└── scripts/
    └── install.sh               One-time RPi setup script
```
