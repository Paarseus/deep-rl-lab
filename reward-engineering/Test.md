# Reward Engineering for CARLA Simulator + Gymnasium

**Author:** Alexander Assal  
**Date:** May 31, 2025  

---

## 🧠 Goal

Design a reward structure for a Gymnasium-compatible RL agent in CARLA that encourages:
1. Safe, collision-free driving  
2. Compliance with traffic rules (lane keeping, speed limits)  
3. Efficient progress toward a predefined goal location  

Sensors used (conceptually):
- **Collision sensor** (`sensor.other.collision`) to detect any collisions.  
- **Lane invasion sensor** (`sensor.other.lane_invasion`) to detect when the vehicle crosses lane markings.  
- **Vehicle velocity** (via `ego_vehicle.get_velocity()`) to track speed.  
- **Map/waypoint queries** (`world.get_map().get_waypoint(location)`) to compute distance to lane center and distance to goal.  
- **(Optional)** IMU (`sensor.other.imu`) for smoothness/jerk, semantic segmentation camera (`sensor.camera.semantic_segmentation`) for pedestrian/traffic-light detection.

---

## 🛣️ Key Events & Suggested Reward Values

| Event                            | Description                                                | Reward                                           |
|----------------------------------|------------------------------------------------------------|--------------------------------------------------|
| **Reaching Goal**                | Agent arrives within 2 m of the goal location               | **+100** (episode terminates)                    |
| **Progress Toward Goal**         | Reduction in Euclidean distance to goal since last step     | `+ (Δd / initial_distance) × 100`                 |
| **Lane-Keeping (Centered)**      | Lateral distance to lane center ≤ 3 m: bonus proportional   | `+ (3.0 – dist_center)` (∈ [0, +3])               |
| **Target-Speed Proximity**       | Current speed near 22 m/s: higher bonus when closer         | `+ max(0, 10 – |speed – 22|)` (∈ [0, +10])         |

- **Δd**: previous_distance_to_goal – current_distance_to_goal  
- **initial_distance**: distance from start to goal at episode start  

---

## ❌ Negative Reward Events

| Event                            | Description                                                | Reward                                           |
|----------------------------------|------------------------------------------------------------|--------------------------------------------------|
| **Collision**                    | Any collision detected by `sensor.other.collision`         | **–100** (episode terminates)                    |
| **Lane Crossing**                | Crossing any lane marking (`sensor.other.lane_invasion`)   | **–50**                                          |
| **Off-Road (dist_center > 3 m)** | Lateral distance to lane center exceeds 3 m                | **–50**                                          |
| **Below Minimum Speed**          | Current speed < 15 m/s (when not stopped at red/etc.)      | **–50**                                          |
| **Above Maximum Speed**          | Current speed > 25 m/s                                      | **–1** per timestep                              |
| **Timeout**                      | Episode length exceeds MAX_TIMESTEPS (e.g., 7500 steps)     | **–100** (episode terminates)                    |

---

## 🧪 Additional Considerations

- **Smoothness (Optional)**  
  - Use the IMU sensor (`sensor.other.imu`) to compute acceleration and jerk.  
  - Penalize large jerk: for example, `– (|current_acceleration – previous_acceleration| / max_jerk) × 5`, clipped to a minimum of –5 per timestep.  
  - Penalize abrupt steering changes: if steering angle rate > 30°/s, apply up to –5.

- **Traffic-Light Compliance (Optional)**  
  - Use a semantic segmentation camera (`sensor.camera.semantic_segmentation`) to identify red lights.  
  - If approaching a red light and failing to stop before the line (detected via waypoint queries and camera), apply –20.  
  - If correctly stopping at a red light for ≥1 s, bonus +5.

- **Pedestrian Yielding (Optional)**  
  - Detect pedestrians in crosswalks via semantic segmentation or semantic lidar.  
  - If agent yields (speed < 0.5 m/s) when pedestrian is present in crosswalk: +10.  
  - If agent passes within 1.5 m of pedestrian in crosswalk without stopping: –30.

- **Waypoint Advancement (Optional)**  
  - Precompute a route of waypoints from start to goal.  
  - Each time the agent enters the next waypoint region: +5 per waypoint crossed.

---

## 🗂️ Notes for Implementation

1. **Sensor Registration**  
   - In `CarlaEnv.__init__()`, create one `SensorInterface` instance.  
   - For each required sensor, call:  
     ```yaml
     SensorManager.spawn(
       name:            "<sensor_name>",
       attributes:      { "type": "<blueprint>", "transform": "<x,y,z,roll,pitch,yaw>", … },
       interface:       sensor_interface,
       parent:          ego_vehicle
     )
     ```
   - Example:  
     ```yaml
     name: "collision_sensor"
     attributes:
       type: "sensor.other.collision"
       transform: "0,0,0,0,0,0"
     ```

2. **Pulling Sensor Data**  
   - In each `step()`, after `world.tick()`, call:  
     ```python
     sensor_data = self.sensor_interface.get_data()
     ```
   - Check if `"collision_sensor"` or `"lane_invasion_sensor"` are keys in `sensor_data` to detect events.
   - Retrieve streaming data (IMU, GNSS, semantic camera) via their sensor names as needed.

3. **Reward Calculation Order**  
   1. **Collision** → apply **–100**, set `done=True`, return immediately.  
   2. **Lane Crossing** → if present, apply **–50**; else compute lane-keeping bonus.  
   3. **Speed Regulation** → apply **–50** if speed < 15 m/s; **–1** per step if speed > 25 m/s; add proximity bonus for closeness to 22 m/s.  
   4. **Distance to Lane Center** → same as lane-keeping bonus/penalty above.  
   5. **Progress Toward Goal** → compute `delta_d`; if positive, apply `(delta_d / initial_distance) × 100`; if `distance < 2 m`, apply **+100** and terminate.  
   6. **Time Penalty** → apply **–1** per step; if `elapsed_steps ≥ MAX_TIMESTEPS`, apply **–100**, set `done=True`.  

4. **Observation Design (Example)**  
   - A simple observation vector:  
     1. `current_speed` (scalar)  
     2. `distance_to_goal` (scalar)  
     3. `distance_to_center` (scalar)  
   - Optionally augment with semantic segmentation image or GNSS coordinates.

5. **Episode Reset**  
   - Destroy sensors (`sensor_interface.destroy()`) and ego vehicle; respawn both.  
   - Reset counters: `elapsed_steps=0`, `previous_distance_to_goal=None`, `initial_distance_to_goal=None`.

6. **Tuning Recommendations**  
   - Adjust reward magnitudes for stable learning:  
     - If agent rarely reaches goal, increase **+100** to **+200**.  
     - If lane-keeping bonus is too small, multiply `(3 – dist_center)` by 2 or 5.  
     - If time penalty is too weak, increase from **–1** to **–2** per step.  

---

## 7. Reward Summary Tables

### Positive Rewards

| Condition                           | Reward                                                    |
|-------------------------------------|-----------------------------------------------------------|
| Reaching Goal                       | **+100** (ends episode)                                   |
| Progress Toward Goal                | `+ (Δd / initial_distance) × 100` (only if Δd > 0)         |
| Lane-Keeping (dist_center ≤ 3 m)    | `+ (3 – dist_center)` (∈ [0, +3])                          |
| Target-Speed Proximity (|speed–22|) | `+ max(0, 10 – |speed – 22|)` (∈ [0, +10])                 |
| (Optional) Stop at Red Light        | +5 (if fully stopped ≥1 s before red light)              |
| (Optional) Yield to Pedestrian      | +10 (if stopped when pedestrian in crosswalk)            |
| (Optional) Waypoint Advancement     | +5 per waypoint crossed along precomputed route          |

### Negative Rewards

| Condition                                 | Reward                          |
|-------------------------------------------|---------------------------------|
| Collision                                 | **–100** (ends episode)         |
| Lane Crossing / Off-Road (dist_center >3) | **–50**                         |
| Speed < MIN_SPEED (15 m/s)                | **–50**                         |
| Speed > MAX_SPEED (25 m/s)                | **–1** per step                 |
| Timeout (elapsed_steps ≥ 7500)            | **–100** (ends episode)         |
| (Optional) High Jerk or Abrupt Steering   | Up to –5 per event              |
| (Optional) Running Red Light              | **–20**                         |
| (Optional) Passing Pedestrian Without Yield| **–30**                        |

---

## 8. Integration Notes

- **Sensor Mapping:**  
  - `collision_sensor` → collision events.  
  - `lane_invasion_sensor` → lane crossings.  
  - IMU (`imu_sensor`) → acceleration & jerk (optional smoothness).  
  - GNSS (`gnss_sensor`) or direct `get_transform()` → position for distance-to-goal.  
  - Semantic segmentation (`sem_seg_camera`) → pedestrian/traffic-light detection (optional).

- **Step-by-Step Flow:**  
  1. **Apply control** (steer, throttle, brake).  
  2. **Tick CARLA** (`world.tick()`).  
  3. **Retrieve** `sensor_data = sensor_interface.get_data()`.  
  4. **Check collision** → –100 & terminate.  
  5. **Check lane invasion** → –50 or lane-keeping bonus.  
  6. **Compute speed-based** penalties/bonuses.  
  7. **Compute progress-to-goal** reward; if reached → +100 & terminate.  
  8. **Apply time penalty** (–1); if timeout → –100 & terminate.  
  9. **Return** `(observation, reward, done, info)`.

- **Observations & Actions:**  
  - **Action space:**  
    - Continuous: `[steer ∈ (–1,1), throttle ∈ (0,1), brake ∈ (0,1)]`.  
    - Discrete: map indices to predefined controls.  
  - **Observation space example:**  
    ```  
    [ current_speed, 
      distance_to_goal, 
      distance_to_center ]  
    ```

- **Episode Reset:**  
  - Destroy sensors & ego vehicle; respawn both.  
  - Reset counters: `elapsed_steps = 0`, `previous_distance_to_goal = None`, `initial_distance_to_goal = None`.

---

## 9. References

1. **CARLA Simulator Documentation (Python API):**  
   https://carla.readthedocs.io/en/latest/python_api/  

2. **Gymnasium API Reference:**  
   https://gymnasium.farama.org/  

3. **CARLA Sensor Tutorials:**  
   - https://carla.readthedocs.io/en/latest/tuto_sensors/  
   - https://carla.readthedocs.io/en/latest/tuto_Gym_env/  

4. **Global Route Planning (Optional):**  
   - `agents/navigation/global_route_planner.py`  
   - `agents/navigation/global_route_planner_dao.py`
