# Adding Pure Pursuit to ROSMASTER R2 to act as f1tenth car 
## Full Run Manual (Pure Pursuit + AMCL)  
### F1TENTH-Style Closed-Loop Racing Workflow

---

## System Overview

**Robot:** Yahboom ROSMASTER R2  
**Computer:** Jetson Xavier NX  
**ROS:** ROS1 Melodic (Python 2.7)

**Workspace:**
```
/home/jetson/ROS/R2/yahboomcar_ws
```

**Symlink:**
```
/home/jetson/yahboomcar_ws
```

This README explains **every phase** of running the ROSMASTER R2 — from **manual joystick driving** to **closed-loop autonomous Pure Pursuit racing** using:

- AMCL localization  
- FTG safety  
- `twist_mux` arbitration  
- Deadman button (Y)

The workflow mirrors how an **F1TENTH racecar** is operated.

---

## Table of Contents

1. Safety First  
2. Big Picture (How Motion Flows)  
3. Rules You Must Follow  
4. Environment Setup (Always Do This)  
5. Phase 1 – Manual Drive (Teleop Only)  
6. Phase 2 – Mapping (Cartographer)  
7. Phase 3 – Localization (AMCL Only)  
8. Phase 4 – Record a Lap (Rosbag)  
9. Phase 5 – Raceline Generation (Offline)  
10. Phase 6 – Visualize Raceline in RViz  
11. Phase 7 – Autonomous Driving (Pure Pursuit + FTG Safety)  
12. Race Day Launch Order (Fast)  
13. Pure Pursuit Tuning Table  
14. Troubleshooting Decision Tree  
15. AMCL ON vs OFF Rules  

---

## 1. Safety First

- Put the robot on a stand for first tests  
- Keep a finger near the power switch  
- Start slow  
- **Never test near people, pets, stairs, or traffic**

---

## 2. Big Picture (How Motion Flows)

```
Joystick        -> /cmd_vel_teleop  ┐
Pure Pursuit    -> /cmd_vel_auto    ├─> twist_mux -> /cmd_vel -> Motor Driver
FTG Safety      -> /cmd_vel_safety  ┘
```

### Priority inside `twist_mux` (highest wins)

1. FTG Safety  
2. Pure Pursuit (only when A button held)  
3. Teleop joystick (always allowed)

---

## 3. Rules You Must Follow

- ❌ Mapping and localization **never run together**  
- ❌ Pure Pursuit **never runs during mapping**  
- ✅ RViz is **viewer only**  
- ✅ When changing phases: **STOP old nodes first → START new ones**

---

## 4. Environment Setup (Always Do This)

Run in **every new terminal**:

```bash
source /opt/ros/melodic/setup.bash
source ~/ROS/R2/yahboomcar_ws/devel/setup.bash
```

---

## 5. Phase 1 – Manual Drive (Teleop Only)

### Goal
Verify motors, joystick, LiDAR, and `twist_mux`.

### RUN
```bash
roslaunch yahboomcar_nav laser_bringup.launch
```

### DO NOT RUN
- Cartographer  
- AMCL  
- Pure Pursuit  

### RViz
- Fixed Frame: `odom`  
- Add: LaserScan (`/scan`), TF, RobotModel  

### Checks
```bash
rostopic hz /scan
rostopic echo -n 1 /cmd_vel
```

Robot must drive using the joystick.

---

## 6. Phase 2 – Mapping (Cartographer)

### Goal
Create a map.

### STOP FIRST
- AMCL  
- Pure Pursuit  

### RUN
```bash
roslaunch yahboomcar_nav yahboomcar_map.launch map_type:=cartographer
```

### RViz
- Fixed Frame: `map`  
- Add: Map, LaserScan, TF, RobotModel  

### Save Map
```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/home2
```

Stop mapping after saving.

---

## 7. Phase 3 – Localization (AMCL Only)

### Goal
Robot knows where it is on the saved map.

### STOP
- Cartographer  

### RUN
```bash
roslaunch r2_amcl_localization amcl_only.launch map:=/home/jetson/maps/home2.yaml
```

### RViz
- Fixed Frame: `map`  
- Add: Map, LaserScan, TF, RobotModel  
- Use **2D Pose Estimate once**

### Check
```bash
rostopic echo -n 1 /amcl_pose
```

---

## 8. Phase 4 – Record a Lap (Rosbag)

### Goal
Record a clean lap for raceline generation.

### KEEP RUNNING
- Bringup  
- LiDAR  
- AMCL  

### DO NOT RUN
- Pure Pursuit  

### Record Bag
```bash
mkdir -p ~/bags
rosbag record -O ~/bags/amcl_lap_01.bag   /amcl_pose /tf /tf_static /scan /map
```

Drive one smooth lap, then stop with `Ctrl+C`.

---

## 9. Phase 5 – Raceline Generation (Offline)

### Goal
Convert bag → raceline YAML.

```bash
mkdir -p ~/paths
rosrun r2_raceline_pp bag_to_raceline_yaml_amcl.py   --bag /home/jetson/bags/amcl_lap_01.bag   --pose /amcl_pose   --out /home/jetson/paths/raceline_amcl_01.yaml   --frame_id map   --every 1   --min_dist 0.02   --resample 0.05   --close_loop
```

### REQUIRED EDIT
```bash
nano ~/paths/raceline_amcl_01.yaml
```

- `frame_id: map`  
- **Last waypoint must equal first waypoint (loop closure)**

---

## 10. Phase 6 – Visualize Raceline in RViz

### Terminals

**Terminal 1 — Sensors**
```bash
roslaunch yahboomcar_nav laser_bringup.launch
```

**Terminal 2 — AMCL**
```bash
roslaunch r2_amcl_localization amcl_only.launch map:=/home/jetson/maps/home2.yaml
```

**Terminal 3 — Raceline → Path**
```bash
roslaunch r2_raceline_pp raceline_to_path.launch   raceline:=/home/jetson/paths/raceline_amcl_01.yaml
```

**Terminal 4 — Safety Mux**
```bash
roslaunch r2_estop_joy safety_mux.launch
```

**Terminal 5 — Pure Pursuit**
```bash
roslaunch r2_raceline_pp pure_pursuit.launch   raceline:=/home/jetson/paths/raceline_amcl_01.yaml
```

### RViz View
```bash
roslaunch r2_raceline_pp amcl_raceline_rviz_fixed.launch
```

**Enable button:** Y (index 4 → `/pp_enable`)

---

## 11. Pure Pursuit Tuning

File:
```
r2_raceline_pp/config/pure_pursuit.yaml
```

Rules:
- Oscillation → increase lookahead  
- Corner cutting → decrease lookahead  
- Overshoot → lower speed  

---

## 12. Race Day Launch Order (Fast)

1. `laser_bringup.launch`  
2. `amcl_only.launch`  
3. `raceline_to_path.launch`  
4. `safety_mux.launch`  
5. `pure_pursuit.launch`  
6. Hold **A** → GO  

---

## 13. Pure Pursuit Tuning Table

| Speed (m/s) | Lookahead (m) | Use Case | If Wrong |
|------------:|---------------|----------|----------|
| 0.20 | 0.35–0.45 | First test / stand | Twitchy or wide |
| 0.30 | 0.45–0.60 | Slow indoor | Oscillation |
| 0.40 | 0.60–0.80 | Normal indoor | S-wiggle |
| 0.50 | 0.80–1.00 | Fast indoor | Under-steer |
| 0.60 | 1.00–1.20 | Only if AMCL solid | Unstable |

Rules:
- More speed = more lookahead  
- Tight loop → cap lookahead, slow down  
- Open loop → 1.0–1.5 m works  

---

## 14. Troubleshooting Decision Tree (No Guessing)

### 1) Nothing moves
- Check teleop  
- Check A button  
- Check PP output  
- Check `twist_mux` output  

### 2) Path looks polygonal
- Verify YAML path loaded  
- Check waypoint count  
- Verify RViz topic  

### 3) AMCL arrow missing
- Use PoseWithCovariance  
- Check `/amcl_pose` Hz  

### 4) S-wiggle
- Increase lookahead  
- Reduce speed  
- Increase resample spacing  

### 5) Cuts corners
- Decrease lookahead  
- Slow down  

### 6) Overshoots
- Increase lookahead  
- Check steering limits  

### 7) AMCL jumps
- Wait for stability  
- Set 2D Pose  
- Then record  

### 8) Two `twist_mux` nodes
```bash
rosnode list | grep twist_mux
```
Kill one.

---

## 15. AMCL ON vs OFF

| Phase | AMCL |
|------|------|
| Manual Drive | OFF |
| Mapping | OFF |
| Localization | ON |
| Bag Recording | ON |
| Pure Pursuit | ON |

---

## End

You now have a **full, repeatable, F1TENTH-style Pure Pursuit workflow** for the ROSMASTER R2.
