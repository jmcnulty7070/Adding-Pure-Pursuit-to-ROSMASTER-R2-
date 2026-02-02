# ROSMASTER R2 – Pure Pursuit Full Run Manual (Expanded)
*(Pure Pursuit + AMCL + FTG Safety, F1TENTH-style racing)*

This document is the **authoritative, end-to-end manual** for running the
**Yahboom ROSMASTER R2** in closed-loop autonomous racing mode using **Pure Pursuit**
with **AMCL localization**, **FTG safety**, and a **Y-button deadman**.

---

## System Information

- **Robot:** Yahboom ROSMASTER R2  
- **Computer:** Jetson Xavier NX  
- **ROS:** ROS1 Melodic (Python 2.7)

**Workspace**
```text
/home/jetson/ROS/R2/yahboomcar_ws
```
Symlink:
```text
/ home/jetson/yahboomcar_ws
```

---

## Core Safety Rule (Read This First)

> **The robot may ONLY drive autonomously while the Y button is held.**
>
> Releasing Y immediately returns control to joystick teleop.

---

## Motion Architecture (Mental Model)

```text
Joystick        -> /cmd_vel_teleop  ┐
Pure Pursuit    -> /cmd_vel_auto    ├─> twist_mux -> /cmd_vel -> Motor Driver
FTG Safety      -> /cmd_vel_safety  ┘
```

**twist_mux priority**
1. FTG Safety (always wins)
2. Pure Pursuit (only while holding Y)
3. Teleop joystick

---

## Phase Rules (DO NOT VIOLATE)

- ❌ Cartographer and AMCL never run together
- ❌ Pure Pursuit never runs during mapping
- ❌ Two twist_mux nodes must never run together
- ✅ RViz is visualization only
- ✅ Stop old nodes before starting new phases

---

## Phase 1 – Manual Drive (Teleop Only)

**Purpose**
Verify hardware, joystick mapping, LiDAR, and motor driver.

**Run**
```bash
roslaunch yahboomcar_nav laser_bringup.launch
```

**Do NOT run**
- Cartographer
- AMCL
- Pure Pursuit

**RViz**
- Fixed frame: `odom`
- Displays: LaserScan, TF, RobotModel

**Verify**
```bash
rostopic hz /scan
rostopic echo -n 1 /cmd_vel
```

Robot must move smoothly with joystick input.

---

## Phase 2 – Mapping (Cartographer)

**Purpose**
Create a static map of the environment.

**Stop**
- AMCL
- Pure Pursuit

**Run**
```bash
roslaunch yahboomcar_nav yahboomcar_map.launch map_type:=cartographer
```

**RViz**
- Fixed frame: `map`
- Displays: Map, LaserScan, TF

**Save Map**
```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/home2
```

Stop Cartographer immediately after saving.

---

## Phase 3 – Localization (AMCL)

**Purpose**
Localize robot on the saved map.

**Stop**
- Cartographer

**Run**
```bash
roslaunch r2_amcl_localization amcl_only.launch map:=/home/jetson/maps/home2.yaml
```

**RViz**
- Fixed frame: `map`
- Use **2D Pose Estimate ONCE**
- Wait 10–20 seconds for convergence

**Check**
```bash
rostopic hz /amcl_pose
```

---

## Phase 4 – Record a Clean Lap (Rosbag)

**Purpose**
Capture a smooth lap for raceline generation.

**Keep running**
- Bringup
- LiDAR
- AMCL

**Do NOT run**
- Pure Pursuit

**Record**
```bash
mkdir -p ~/bags
rosbag record -O ~/bags/amcl_lap_01.bag   /amcl_pose /tf /tf_static /scan /map
```

Drive **one clean, smooth lap**. Avoid stops and jerks.

---

## Phase 5 – Raceline Generation (Offline)

**Purpose**
Convert rosbag into a smooth closed-loop path.

```bash
mkdir -p ~/paths

rosrun r2_raceline_pp bag_to_raceline_yaml_amcl.py   --bag /home/jetson/bags/amcl_lap_01.bag   --pose /amcl_pose   --out /home/jetson/paths/raceline_amcl_01.yaml   --frame_id map   --every 1   --min_dist 0.02   --resample 0.05   --close_loop
```

**Mandatory edit**
```bash
nano ~/paths/raceline_amcl_01.yaml
```

- `frame_id: map`
- First waypoint == last waypoint

---

## Phase 6 – Visualize Raceline (Dry Run)

**Terminal order**
```text
1. laser_bringup.launch
2. amcl_only.launch
3. raceline_to_path.launch
4. safety_mux.launch
5. pure_pursuit.launch
```

**RViz**
```bash
roslaunch r2_raceline_pp amcl_raceline_rviz_fixed.launch
```

Confirm:
- Path overlays map correctly
- Robot arrow aligns with path

---

## Phase 7 – Autonomous Racing

**Engage**
- Hold **Y**
- Robot follows raceline
- FTG overrides when obstacles appear

Release **Y** instantly to regain teleop control.

---

## Pure Pursuit Tuning Table

| Speed (m/s) | Lookahead (m) |
|------------|---------------|
| 0.20 | 0.35–0.45 |
| 0.30 | 0.45–0.60 |
| 0.40 | 0.60–0.80 |
| 0.50 | 0.80–1.00 |
| 0.60 | 1.00–1.20 |

Rules:
- Wiggle → increase lookahead
- Corner cut → decrease lookahead
- Overshoot → reduce speed

---

## Final Truths

- Hold **Y** = autonomous
- Release **Y** = safe
- One mux only
- One map at a time
- One clean lap beats ten bad ones

**End of Full Manual**
