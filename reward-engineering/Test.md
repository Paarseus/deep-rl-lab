## Reward Engineering for CARLA Simulator

**Author:** Alexander Assal  
**Date:** May 31, 2025  

---

## Table of Contents

1. [Overview](#overview)  
2. [Directory & File Structure](#directory--file-structure)  
3. [Configuration Parameters (`Config.py`)](#configuration-parameters-configpy)  
4. [Sensor Setup & Data Retrieval](#sensor-setup--data-retrieval)  
   1. [Sensors Used](#sensors-used)  
   2. [Spawning & Registering](#spawning--registering)  
   3. [Pulling Sensor Data](#pulling-sensor-data)  
5. [High-Level Objectives](#high-level-objectives)  
6. [Reward Components](#reward-components)  
   1. [Collision Detection](#collision-detection)  
   2. [Lane-Keeping & Deviation](#lane-keeping--deviation)  
   3. [Speed Regulation](#speed-regulation)  
   4. [Distance to Lane Center](#distance-to-lane-center)  
   5. [Progress Toward Goal](#progress-toward-goal)  
   6. [Pedestrian Interaction](#pedestrian-interaction)  
   7. [Time/Efficiency](#timeefficiency)  
7. [Reward Summary Tables](#reward-summary-tables)  
8. [Integration Notes](#integration-notes)  
9. [References](#references)  

---

## 1. Overview

This document defines a reward scheme for a Gymnasium-compatible RL agent in CARLA that incentivizes:

- Safe, collision-free driving  
- Adherence to lane and speed rules  
- Yielding to pedestrians and obeying traffic signals  
- Efficient progress toward a predefined goal  

It leverages:

- **`Sensor_interface.py`** to buffer sensor data  
- **`Sensor_manager.py`** to spawn required sensors  
- **`Config.py`** to store all numeric constants  

Sensors feed specific reward terms without extraneous code examples.

---

## 2. Directory & File Structure

/project_root/
│
├── envs/
│   └── carla_env.py            # Gymnasium wrapper (“CarlaEnv”)
│
├── sensors/
│   ├── Sensor.py               # BaseSensor, CarlaSensor, each sensor class
│   ├── Sensor_interface.py     # Buffers continuous and event data
│   └── Sensor_manager.py       # spawn(name, attributes, interface, parent)
│
├── Config.py                   # All simulation & reward constants
├── reward_engineering.md       # ← (This file)
├── Traffic_config.py           # Traffic spawn parameters
├── Traffic_manager.py          # NPC traffic generation
└── Connection_manager.py       # CARLA client connection

---

## 3. Configuration Parameters (`Config.py`)

```python
# Connection & Episode
HOST = "localhost"
PORT = 2000
TIMEOUT = 30.0
EPISODE_LENGTH = 120        # seconds
MAX_TIMESTEPS = 7500        # steps per episode

# Vehicle & Traffic
CAR_NAME = 'vehicle.mini.cooper'
NUMBER_OF_VEHICLES = 10
NUMBER_OF_PEDESTRIANS = 30

# Action Space
CONTINUOUS_ACTION = True

# Camera
RGB_CAMERA = 'sensor.camera.rgb'
SEMANTIC_CAMERA = 'sensor.camera.semantic_segmentation'
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 80
CAMERA_FOV = 125

# Reward Parameters
TARGET_SPEED = 22.0                # m/s
MAX_SPEED = 25.0                   # m/s
MIN_SPEED = 15.0                   # m/s
MAX_DISTANCE_FROM_CENTER = 3.0     # meters

COLLISION_REWARD = -100            # flat collision penalty
LANE_DEVIATION_REWARD = -50        # lane crossing or off-road penalty
SLOW_SPEED_REWARD = -50            # speed < MIN_SPEED penalty
SPEED_PENALTY = -1                 # per-step penalty if speed > MAX_SPEED

PEDESTRIAN_YIELD_BONUS = +10       # yield to pedestrian in crosswalk
PEDESTRIAN_VIOLATION_PENALTY = -30 # passing too close to pedestrian
RED_LIGHT_BONUS = +5               # stopping at red light for ≥1s
RED_LIGHT_VIOLATION_PENALTY = -20  # running a red light


⸻

4. Sensor Setup & Data Retrieval

4.1 Sensors Used
	•	Collision (sensor.other.collision)
	•	Type: Event
	•	Parsed Data: [other_actor, impulse_value]
	•	LaneInvasion (sensor.other.lane_invasion)
	•	Type: Event
	•	Parsed Data: [transform, crossed_lane_markings]
	•	Imu (sensor.other.imu)
	•	Type: Streaming
	•	Parsed Data: [accX, accY, accZ, gyroX, gyroY, gyroZ, compass]
	•	Gnss (sensor.other.gnss)
	•	Type: Streaming
	•	Parsed Data: [latitude, longitude, altitude]
	•	CameraSemanticSegmentation (sensor.camera.semantic_segmentation)
	•	Type: Streaming
	•	Parsed Data: H×W×3 uint8 array (pixel-wise class colors)

4.2 Spawning & Registering

In CarlaEnv.__init__():
	1.	Initialize self.sensor_interface = SensorInterface().
	2.	Spawn ego vehicle via CARLA API.
	3.	Spawn sensors with SensorManager.spawn(name, attributes, self.sensor_interface, self.ego_vehicle):
	•	Collision Sensor

name: "collision_sensor"
attributes:
  type: "sensor.other.collision"
  transform: "0,0,0,0,0,0"


	•	LaneInvasion Sensor

name: "lane_invasion_sensor"
attributes:
  type: "sensor.other.lane_invasion"
  transform: "0,0,0,0,0,0"


	•	IMU Sensor

name: "imu_sensor"
attributes:
  type: "sensor.other.imu"
  transform: "0,0,0,0,0,0"


	•	GNSS Sensor

name: "gnss_sensor"
attributes:
  type: "sensor.other.gnss"
  transform: "0,0,2,0,0,0"


	•	Semantic Segmentation Camera

name: "sem_seg_camera"
attributes:
  type: Config.SEMANTIC_CAMERA           # "sensor.camera.semantic_segmentation"
  transform: "1.5,0,2.4,0,0,0"
  image_size_x: str(Config.IMAGE_WIDTH)
  image_size_y: str(Config.IMAGE_HEIGHT)
  fov: str(Config.CAMERA_FOV)



4.3 Pulling Sensor Data

In CarlaEnv.step(action) after world.tick():

sensor_data = self.sensor_interface.get_data()

	•	Event sensors (e.g., "collision_sensor", "lane_invasion_sensor") appear only if an event happened.
	•	Streaming sensors (e.g., "imu_sensor", "gnss_sensor", "sem_seg_camera") return exactly one entry each call.

⸻

5. High-Level Objectives
	1.	Reach a Predefined Goal Location
	•	A goal_location is chosen (carla.Location).
	•	Goal reached if ego_vehicle.get_transform().location.distance(goal_location) < 2 m.
	2.	Avoid Collisions
	3.	Maintain Lane Position (stay within 3 m of lane center)
	4.	Regulate Speed (stay near 22 m/s, never below 15 m/s or above 25 m/s)
	5.	Yield to Pedestrians in crosswalks
	6.	Obey Red Lights (stop fully before red for ≥ 1 s)
	7.	Efficient Progress (minimize steps to goal)

⸻

6. Reward Components

6.1 Collision Detection
	•	Sensor: "collision_sensor"
	•	Trigger: event in sensor_data
	•	Reward:
	•	–100 (Config.COLLISION_REWARD) and terminate episode

6.2 Lane-Keeping & Deviation
	•	Sensors: "lane_invasion_sensor" and map API
	•	Trigger:
	•	If "lane_invasion_sensor" present: –50 (Config.LANE_DEVIATION_REWARD)
	•	Else: compute dist_center = lateral distance to lane center:

waypoint = world.get_map().get_waypoint(current_loc, project_to_road=True)
center_loc = waypoint.transform.location
dist_center = current_loc.distance(center_loc)

	•	If dist_center ≤ 3.0 m:
	•	Bonus: 3.0 – dist_center (∈ [0, +3])
	•	If dist_center > 3.0 m:
	•	Penalty: –50

6.3 Speed Regulation
	•	Sensor: none (use ego_vehicle.get_velocity())
	•	Compute: current_speed = √(v.x² + v.y² + v.z²) (m/s)
	•	Reward/Penalty:
	•	If current_speed < 15.0 m/s: –50 (Config.SLOW_SPEED_REWARD)
	•	If current_speed > 25.0 m/s: –1 (Config.SPEED_PENALTY) per step
	•	Speed-proximity bonus:

speed_error = abs(current_speed – 22.0)  # TARGET_SPEED
bonus = max(0, 10 – speed_error)         # ∈ [0, +10]
reward += bonus



6.4 Distance to Lane Center
	•	Sensor: none (use map API, same as lane-keeping)
	•	Reward/Penalty: identical to lane-keeping bonus/penalty above.

6.5 Progress Toward Goal
	•	Sensor: none (use ego_vehicle.get_transform().location)
	•	Procedure:
	1.	On first step:

prev_dist = current_loc.distance(goal_location)
init_dist = prev_dist


	2.	Each step:

cur_dist = current_loc.distance(goal_location)
delta_d = prev_dist – cur_dist
if delta_d > 0:
    reward += (delta_d / init_dist) * 100  # normalized to [0, +100]
prev_dist = cur_dist


	3.	If cur_dist < 2 m:
	•	Reward: +100; terminate episode

6.6 Pedestrian Interaction
	•	Sensor: "sem_seg_camera" to detect pedestrians (semantic class color)
	•	Procedure:
	1.	Identify pixels corresponding to pedestrians in front of vehicle.
	2.	Compute bounding area or distance to nearest pedestrian using semantic segmentation (or semantic lidar).
	3.	If pedestrian is in crosswalk and current_speed < 0.5 m/s:
	•	Reward: +10 (Config.PEDESTRIAN_YIELD_BONUS)
	4.	If pedestrian in crosswalk and current_speed ≥ 0.5 m/s or vehicle passes within 1.5 m of pedestrian:
	•	Penalty: –30 (Config.PEDESTRIAN_VIOLATION_PENALTY)

6.7 Red Light Compliance
	•	Sensor: "sem_seg_camera" to identify traffic light pixels
	•	Procedure:
	1.	Detect red traffic light ahead via semantic segmentation.
	2.	Compute distance to stop line via waypoint query.
	3.	If red light and distance_to_stop_line < 0 and current_speed > 1 m/s:
	•	Penalty: –20 (Config.RED_LIGHT_VIOLATION_PENALTY)
	4.	If red light is present and distance_to_stop_line > 0 and current_speed < 0.5 m/s for ≥ 1 s:
	•	Reward: +5 (Config.RED_LIGHT_BONUS)

6.8 Time/Efficiency
	•	Sensor: none (use internal self.elapsed_steps)
	•	Reward/Penalty:
	•	Per-step: –1 (Config.SPEED_PENALTY)
	•	If elapsed_steps ≥ MAX_TIMESTEPS: –100 and terminate episode

⸻

7. Reward Summary Tables

Positive Rewards

Condition	Reward
Reaching Goal	+100 (distance < 2 m; terminate)
Progress Toward Goal	+ (Δd / init_dist) × 100
Lane-Keeping (dist_center ≤ 3 m)	+ (3.0 – dist_center) (∈ [0, +3])
Target-Speed Proximity	`+ max(0, 10 –
Yield to Pedestrian	+10 (speed < 0.5 m/s when pedestrian present)
Stopping at Red Light	+5 (stopped ≥1 s before red light)

Negative Rewards

Condition	Reward
Collision	–100 (terminate)
Lane Crossing / Off-Road (dist_center>3)	–50
Speed < MIN_SPEED (15 m/s)	–50
Speed > MAX_SPEED (25 m/s)	–1 per step
Passing Pedestrian Without Yield	–30
Running Red Light	–20
Timeout (steps ≥ 7500)	–100 (terminate)


⸻

8. Integration Notes
	1.	Sensor Retrieval:

sensor_data = self.sensor_interface.get_data()

	•	Check "collision_sensor" or "lane_invasion_sensor" in sensor_data for events.
	•	Streaming sensors ("imu_sensor", "gnss_sensor", "sem_seg_camera") return data every call.

	2.	Reward Calculation Order:
	1.	Collision → –100, set done=True, return.
	2.	Lane Crossing → –50; else compute lane-keeping bonus (+[0–3]).
	3.	Speed Regulation → –50 if <15 m/s; –1 if >25 m/s; add proximity bonus (+[0–10]).
	4.	Distance to Center → identical to lane-keeping logic.
	5.	Progress Toward Goal → normalized distance reduction reward; if goal reached (dist < 2 m) → +100, set done=True, return.
	6.	Pedestrian Interaction → +10 if speed < 0.5 m/s with pedestrian in crosswalk; else –30.
	7.	Red Light Compliance → –20 if running red; +5 if stopped ≥1 s before red.
	8.	Time Penalty → –1 per step; if elapsed_steps ≥ MAX_TIMESTEPS → –100, set done=True, return.
	3.	Observation Example:

[ current_speed, distance_to_goal, dist_center ]

	•	Augment with semantic camera image or GNSS coordinates if needed.

	4.	Action Space:
	•	Continuous: [steer ∈ (–1,1), throttle ∈ (0,1), brake ∈ (0,1)]
	•	Discrete: map indices to preset controls (e.g., throttle only, brake only, left, right, no-op).
	5.	Episode Reset:
	•	Destroy sensors (sensor_interface.destroy()) and ego vehicle; respawn.
	•	Reset counters: elapsed_steps = 0, previous_distance_to_goal = None, initial_distance_to_goal = None.
	6.	Tuning Guidelines:
	•	Increase goal bonus if agent seldom completes.
	•	Scale lane-keeping bonus (e.g., multiply by 5) if center reward too small.
	•	Adjust time penalty (e.g., –2) if agent stalls.

⸻

9. References
	1.	CARLA Simulator Documentation (Python API):
https://carla.readthedocs.io/en/latest/python_api/
	2.	Gymnasium API Reference:
https://gymnasium.farama.org/
	3.	CARLA Sensor Tutorials:
	•	https://carla.readthedocs.io/en/latest/tuto_sensors/
	•	https://carla.readthedocs.io/en/latest/tuto_Gym_env/
	4.	Global Route Planning:
	•	agents/navigation/global_route_planner.py
	•	agents/navigation/global_route_planner_dao.py

