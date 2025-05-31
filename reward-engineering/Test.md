# Reward Engineering for CARLA Simulator + Gymnasium

*Author: Alexander Assal*  
*Date: 31 May 2025*

---

## Table of Contents

1. [Introduction](#introduction)  
2. [Environment Setup & Observations](#environment-setup--observations)  
    1. [CARLA + Gymnasium Integration](#carla--gymnasium-integration)  
    2. [Sensors & Data Streams](#sensors--data-streams)  
    3. [Action Space Definition](#action-space-definition)  
3. [High-Level Objectives](#high-level-objectives)  
4. [Reward Components](#reward-components)  
    1. [Primary (Terminal/Goal) Rewards](#primary-terminalgoal-rewards)  
    2. [Safety & Traffic-Rule Compliance](#safety--traffic-rule-compliance)  
    3. [Lane-Keeping & Drivability](#lane-keeping--drivability)  
    4. [Progress Toward Destination](#progress-toward-destination)  
    5. [Smoothness & Comfort](#smoothness--comfort)  
    6. [Time/Efficiency Penalties](#timeefficiency-penalties)  
5. [Detailed Reward Calculations & Implementation](#detailed-reward-calculations--implementation)  
    1. [Collision Detection & Penalty](#collision-detection--penalty)  
    2. [Traffic Light Handling](#traffic-light-handling)  
    3. [Speed Limit Enforcement](#speed-limit-enforcement)  
    4. [Lane Invasion Handling](#lane-invasion-handling)  
    5. [Pedestrian/Crosswalk Yielding](#pedestriancrosswalk-yielding)  
    6. [Distance & Waypoint-Based Progress](#distance--waypoint-based-progress)  
    7. [Acceleration/Jerk Measurement](#accelerationjerk-measurement)  
    8. [Timeout & Episode Termination](#timeout--episode-termination)  
6. [Pseudo-Code: Reward Computation in `step()`](#pseudo-code-reward-computation-in-step)  
7. [Reward Shaping & Scaling Guidelines](#reward-shaping--scaling-guidelines)  
8. [Tuning Recommendations](#tuning-recommendations)  
9. [Appendix: CARLA API Snippets](#appendix-carla-api-snippets)  
10. [References](#references)

---

## 1. Introduction

Reinforcement Learning (RL) in the CARLA Simulator requires carefully designed reward functions to guide the agent toward safe, efficient, and rule-abiding driving behavior. This document outlines  reward engineering strategy for **Carla** including sensor usage, event detection, and exact numeric scaling. All pseudocode and examples assume a Gymnasium-compatible RL environment wrapping CARLA.  

---

## 2. Environment Setup & Observations

### 2.1 CARLA + Gymnasium Integration

1. **CARLA Version**:  
   - Tested with CARLA v0.10.0
2. **Gymnasium Wrapper**:  
   - Use a custom environment class `CarlaEnv(gym.Env)` that implements `reset()`, `step(action)`, `render()`, and `close()`.  
   - The `step()` method must return `(observation, reward, terminated, truncated, info)` consistent with Gymnasium API.  
3. **Episode Configuration**:  
   - **Spawn Points**: Predefined spawn locations for agent & NPC traffic.  
   - **Weather/Time of Day**: Fixed or randomized; affects visibility but not reward.  

### 2.2 Sensors & Data Streams

1. **RGB Camera (Front-Facing)**  
   - Resolution: 800 × 600 (modify as needed).  
   - Framerate: 20 FPS.  
   - Purpose: Semantic understanding, optional (not directly used in reward).  

2. **Collision Sensor**  
   - Attaches to the ego-vehicle.  
   - Tracks any collision events with vehicles, pedestrians, or static objects.  
   - Provides: Callback with `collision_event.frame`, `actor_type`, and `impulse`.  

3. **Lane Invasion Sensor**  
   - Detects when ego-vehicle crosses lane markings.  
   - Provides: Callback with `lane_type` array (e.g., `Solid`, `Broken`).  

4. **Speedometer (Velocity Measurement)**  
   - Directly accessible via `vehicle.get_velocity()` (returns a `carla.Vector3D`).  
   - Compute scalar speed in m/s:  
     ```python
     vel = vehicle.get_velocity()
     speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
     ```  
5. **Traffic Light Sensor / Traffic Light State**  
   - Query `vehicle.get_traffic_light()` to obtain the traffic light object at front.  
   - Use `traffic_light.get_state()` returning `carla.TrafficLightState.Red/Green/Yellow`.  

6. **Waypoint/Map Information**  
   - Use `world.get_map().get_waypoint(location)` to find nearest waypoint.  
   - Provides `waypoint.lane_id`, `waypoint.transform.location`, and `waypoint.road_id`.  
   - Helps compute distance to next waypoint or target.  

7. **GNSS (Global Positioning)**  
   - Optional: `GnssSensor`, returns latitude & longitude; not strictly required if using CARLA’s waypoint system.  

8. **IMU (Accelerometer/Gyro)**  
   - Optional: For jerk/smoothness penalty.  
   - Returns acceleration and angular velocity.  

### 2.3 Action Space Definition

1. **Continuous Control** (`Box` space)  
   - **Steering**: `float ∈ [-1.0, 1.0]` (left to right).  
   - **Throttle**: `float ∈ [0.0, 1.0]`.  
   - **Brake**: `float ∈ [0.0, 1.0]`.  

2. **Discrete Control** (`Discrete` space)  
   - Example mapping (if using a discrete wrapper):  
     1. Accelerate only  
     2. Brake only  
     3. Steer left + slight throttle  
     4. Steer right + slight throttle  
     5. No operation (coast)  

3. **Action Frequency**  
   - One action per simulation step.  
   - Simulation step = 0.05 seconds (20 Hz) by default.  

---

## 3. High-Level Objectives

1. **Primary Objective: Reach Destination Safely**  
   - Agent must navigate from start waypoint to target waypoint without major infractions.  

2. **Secondary Objectives (Sub-Goals)**  
   1. **Avoid Collisions** with vehicles, pedestrians, and static objects.  
   2. **Comply with Traffic Signals** (stop at red, proceed on green).  
   3. **Maintain Lane Discipline** (avoid lane invasions).  
   4. **Yield to Pedestrians** at crosswalks and zebra crossings.  
   5. **Observe Speed Limits** (no excessive speeding or extreme idling).  
   6. **Drive Smoothly** (minimize sudden acceleration/braking).  

3. **Tertiary Objectives**  
   - Minimize overall travel time without compromising safety.  
   - Provide slight incentives for “driving comfort” (steady accelerations).  

---

## 4. Reward Components

Rewards are broken into **Positive** and **Negative** categories. Each component must be computed at every time-step (`t`) and summed into a scalar `r_t`.  

### 4.1 Primary (Terminal/Goal) Rewards

| Event                      | Reward Value | Description                                                                                  |
|----------------------------|-------------:|----------------------------------------------------------------------------------------------|
| **1. Reaching Goal**       |        **+1000** | Agent successfully arrives at the destination (within ε‐meter radius, e.g., 2 m).           |
| **2. Minimum Time Bonus**  |         +100 | If agent reaches goal under a time threshold (e.g., < 30 seconds).                           |
| **3. Complete Episode End**|            0 | No additional reward if episode ends by timeout or crash; terminal penalties apply separately. |

> **Notes**:  
> - “Reaching Goal” triggers `done = True`.  
> - If agent achieves “Reaching Goal,” skip all subsequent negative time-step penalties.  

### 4.2 Safety & Traffic-Rule Compliance

| Event                              | Reward Value | Description                                                                                                              |
|------------------------------------|-------------:|--------------------------------------------------------------------------------------------------------------------------|
| **1. Collision**                   |        **-1000** | Any collision detected by `CollisionSensor`. (High uniform penalty.)                                                      |
| **2. Running Red Light**           |         **-200**  | Agent crosses a red traffic light boundary (detected by `TrafficLightState` + vehicle location ahead of stop line).     |
| **3. Speeding (Top‐End)**          |          **-50**  | Speed > `SpeedLimit + 10 km/h` for > 1 second (averaged).                                                                |
| **4. Stop Sign Violation**         |          **-100** | Does not stop (speed < 0.5 m/s) within stop‐sign zone for ≥ 2 seconds.                                                    |
| **5. Pedestrian Near‐Miss**        |          **-300** | Ego‐vehicle passes within 1.5 m of pedestrian at designated crosswalk without full stop.                                   |
| **6. Unsafe Overtaking**           |          **-150** | Agent crosses a double‐yellow line to overtake NPC vehicle (detected by `LaneInvasionSensor` + lane-type info).         |

> **Notes**:  
> - All Traffic‐Rule penalties are **immediate** once the event is detected.  
> - If collision occurs simultaneously with another violation in the same timestep, apply collision penalty only (do not stack).  

### 4.3 Lane-Keeping & Drivability

| Event                              | Reward Value        | Description                                                                                                                |
|------------------------------------|--------------------:|----------------------------------------------------------------------------------------------------------------------------|
| **1. Staying in Lane**             |             **+2/second** | Ego‐vehicle center within lane boundaries. Awarded every timestep agent remains fully inside lane.                         |
| **2. Lane Invasion**               |         **-50**      | `LaneInvasionSensor` reports crossing any lane marking (solid or broken).                                                  |
| **3. Lane Off‐Road**               |      **-150**      | More than 50% of bounding‐box off paved lane (e.g., onto shoulder or grass).                                              |
| **4. Lane Change (Valid)**         |             **+10** /per event | If agent signals (optional) and changes to adjacent lane respecting lane markings (tracked via waypoint sequence).         |

> **Notes**:  
> - “Lane Change (Valid)” requires checking previous and current lane IDs via `Waypoint.lane_id`.  
> - “Staying in Lane” is computed at 20 Hz:  
>   ```python
>   if lane_invasion_count == 0:
>       reward += 2 * dt    # dt = simulation delta seconds (e.g., 0.05 s)
>   ```  

### 4.4 Progress Toward Destination

| Event / Metric                         | Reward Value                     | Description                                                                                     |
|----------------------------------------|---------------------------------:|-------------------------------------------------------------------------------------------------|
| **1. Distance Reduction**               |      `+ (Δd / D_total) * 50`     | Normalized reward proportional to reduction in Euclidean distance to goal between t and t−1.   |
| **2. Waypoint Advancement**             |                   **+5** /per waypoint hopped | Each time the agent crosses a waypoint threshold on planned route (use Carla’s global route plan). |
| **3. Orientation Alignment**            |       `+ cos(θ_diff) * 1`        | θ_diff = angle between vehicle’s forward vector and vector to next waypoint.                    |

> **Notes**:  
> - `Δd = previous_distance_to_goal − current_distance_to_goal`.  
> - `D_total` = initial distance from start to goal.  
> - These progress metrics are **continuous** and computed every timestep.  

### 4.5 Smoothness & Comfort

| Metric/Event                        | Reward Value                  | Description                                                                             |
|-------------------------------------|------------------------------:|-----------------------------------------------------------------------------------------|
| **1. Jerk Penalty**                 |         **- (|jerk| / j_max) * 5**  | Jerk = change in acceleration magnitude between t−1 and t. Normalize by max jerk.      |
| **2. Sudden Braking**               |             **-10**             | If brake > 0.9 and throttle ≈ 0 within 0.5 s after throttle > 0.5.                      |
| **3. Rapid Steering**               |       **- (|steering_rate| / 100) * 5** | Compute steering_rate = Δsteering / dt (degrees/s). Penalize if > 30°/s.               |
| **4. Idle Too Long (Block Traffic)**|             **-5/second**        | Speed < 0.5 m/s for > 2 seconds while not at red light or stop sign.                    |

> **Notes**:  
> - Jerk can be computed via IMU sensor or by numerical differentiation of speed.  
> - Clip all smoothness penalties to a minimum of –5 per component to prevent runaway negative rewards.  

### 4.6 Time/Efficiency Penalties

| Event / Condition              | Reward Value      | Description                                                                                        |
|--------------------------------|------------------:|----------------------------------------------------------------------------------------------------|
| **1. Per‐Timestep Penalty**    |       **-1**        | Every environment step, apply a small negative reward to encourage faster completion.             |
| **2. Timeout**                 |      **-500**       | Episode ends due to max episode length (e.g., 10,000 timesteps).                                   |
| **3. Excessive Loitering**     |      **-10/second** | If distance to goal increases for > 5 seconds continuously (circle or reverse driving).            |

> **Notes**:  
> - “Per‐Timestep Penalty” ensures agent does not idly wait.  
> - “Timeout” ensures agent is discouraged from never reaching the goal.  

---

## 5. Detailed Reward Calculations & Implementation

All reward components should be computed **each timestep** in `step()`. Use helper functions to avoid duplication. Below, each subsection explains **how** to detect and compute each reward category using **CARLA’s Python API**.

### 5.1 Collision Detection & Penalty

1. **Setup**  
   - Attach `collision_sensor` to ego‐vehicle:  
     ```python
     collision_sensor = world.spawn_actor(
         blueprint_library.find('sensor.other.collision'),
         carla.Transform(),
         attach_to=ego_vehicle
     )
     collision_sensor.listen(lambda event: collision_events.append(event))
     ```  
   - Maintain a list `collision_events = []` to store collisions in the current timestep.  

2. **Detection in `step()`**  
   ```python
   collision_penalty = 0
   if len(collision_events) > 0:
       # Immediately apply collision penalty once
       collision_penalty = -1000
       done = True
       info['collision'] = True
       # Clear events so penalty is only applied once per collision
       collision_events.clear()
