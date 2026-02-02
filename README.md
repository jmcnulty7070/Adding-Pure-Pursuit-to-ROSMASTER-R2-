# ROSMASTER R2 – Pure Pursuit Raceline (AMCL-Based Racing)

**Full Run Manual – Pure Pursuit + AMCL + FTG Safety**

This repository documents a **complete, repeatable workflow** for running the **Yahboom ROSMASTER R2** like a small F1TENTH-style racecar using:

- AMCL localization (map frame)
- Offline raceline generation from rosbag
- Pure Pursuit path tracking
- FTG safety override
- `twist_mux` with joystick deadman

---

## System Overview

**Robot:** Yahboom ROSMASTER R2  
**Computer:** Jetson Xavier NX  
**ROS:** ROS1 Melodic (Python 2.7)

**Workspace**
```text
/home/jetson/ROS/R2/yahboomcar_ws
```
Symlink:
```text
/home/jetson/yahboomcar_ws
```

---

## Safety First

- Put the robot on a stand for first tests  
- Keep a finger near the power switch  
- Start slow  
- **Never test near people, pets, stairs, or traffic**

---

## Big Picture – How Motion Flows

```text
Joystick        -> /cmd_vel_teleop  ┐
Pure Pursuit    -> /cmd_vel_auto    ├─> twist_mux -> /cmd_vel -> Motor Driver
FTG Safety      -> /cmd_vel_safety  ┘
```

**Priority inside `twist_mux` (highest wins):**
1. FTG Safety  
2. Pure Pursuit (**only when Y button is held**)  
3. Teleop joystick  

---

## Rules You Must Follow

❌ Mapping and localization never run together  
❌ Pure Pursuit never runs during mapping  
✅ RViz is viewer only  
✅ When changing phases: **STOP old nodes → START new ones**

---

## Environment Setup (Always Do This)

Run in **every new terminal**:
```bash
source /opt/ros/melodic/setup.bash
source ~/ROS/R2/yahboomcar_ws/devel/setup.bash
```

---

## Manual Control vs Autonomy

- **Teleop:** Always allowed  
- **Autonomy Enable (Deadman): HOLD Y BUTTON**
- Releasing **Y** immediately returns control to joystick

---

## Mapping, Localization, Raceline, and Racing

Follow the exact phase order:

1. Manual Drive (Teleop only)
2. Mapping (Cartographer)
3. Localization (AMCL)
4. Record Lap (rosbag)
5. Generate Raceline (offline)
6. Visualize in RViz
7. Race with Pure Pursuit + FTG

---

## Raceday Launch Order (Fast)

```text
1. laser_bringup.launch
2. amcl_only.launch
3. raceline_to_path.launch
4. safety_mux.launch
5. pure_pursuit.launch
```

**HOLD Y → GO**

---

## Pure Pursuit Tuning

File:
```text
r2_raceline_pp/config/pure_pursuit.yaml
```

Rules:
- Oscillation → increase lookahead
- Corner cutting → decrease lookahead
- Overshoot → reduce speed

---

## Final Notes

- Autonomy is **always gated by holding Y**
- FTG safety always overrides Pure Pursuit
- If anything behaves strangely, stop and verify **only one twist_mux is running**

---

**This README is authoritative for this repository.**
