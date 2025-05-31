# Reward Engineering for CARLA Simulator

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
   6. [Time/Efficiency](#timeefficiency)  
7. [Reward Summary Tables](#reward-summary-tables)  
8. [Integration Notes](#integration-notes)  
9. [References](#references)  

---

## 1. Overview

A concise reward engineering scheme for a CARLA-based Gymnasium environment. It uses:

- **`Sensor_interface.py`** to buffer all sensor data (streaming and event-based).  
- **`Sensor_manager.py`** to spawn necessary sensors via `spawn(name, attributes, interface, parent)`.  
- **`Config.py`** to centralize all numeric constants (speeds, distances, penalties).  

Rather than full code, this document describes which sensors contribute to each reward term, which `Config.py` constants apply, and how each term is computed conceptually.

---

## 2. Directory & File Structure

/project_root/
│
├── envs/
│   └── carla_env.py            # Gymnasium wrapper (“CarlaEnv”)
│
├── sensors/
│   ├── Sensor.py               # BaseSensor, CarlaSensor, specific sensor classes
│   ├── Sensor_interface.py     # Buffers continuous and event-based data
│   └── Sensor_manager.py       # spawn(name, attributes, interface, parent)
│
├── Config.py                   # Simulation & reward constants
├── reward_engineering.md       # ← (This file)
├── Traffic_config.py           # Traffic spawn parameters
├── Traffic_manager.py          # Spawns NPC traffic
└── Connection_manager.py       # CARLA client connection

---

## 3. Configuration Parameters (`Config.py`)

All constants below appear in `Config.py`. “`Config.XYZ`” refers to these exact names.

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
CONTINUOUS_ACTION = True    # else: Discrete

# Camera (optional)
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

COLLISION_REWARD = -100            # collision penalty
LANE_DEVIATION_REWARD = -50        # lane crossing penalty
SLOW_SPEED_REWARD = -50            # speed < MIN_SPEED penalty
SPEED_PENALTY = -1                 # per-step if speed > MAX_SPEED


⸻

4. Sensor Setup & Data Retrieval

4.1 Sensors Used
	•	Collision (sensor.other.collision)
	•	Event-based: triggers on any collision.
	•	Parsed Data: [other_actor, impulse_value].
	•	LaneInvasion (sensor.other.lane_invasion)
	•	Event-based: triggers when crossing a lane marking.
	•	Parsed Data: [transform, crossed_lane_markings].
	•	Imu (sensor.other.imu) (optional)
	•	Streaming: returns [accX, accY, accZ, gyroX, gyroY, gyroZ, compass].
	•	Use: compute smoothness or jerk penalties if desired.
	•	Gnss (sensor.other.gnss)
	•	Streaming: returns [latitude, longitude, altitude].
	•	Use: approximate global position; primarily use ego_vehicle.get_transform().location.
	•	CameraSemanticSegmentation (sensor.camera.semantic_segmentation) (optional)
	•	Streaming: returns an H×W×3 array for pixel‐wise class IDs.
	•	Use: detect pedestrians or traffic lights if needed.

Minimal Scheme: Only Collision and LaneInvasion are strictly required. Speed and position derive directly from ego_vehicle.get_velocity() and ego_vehicle.get_transform().location.

4.2 Spawning & Registering

Inside CarlaEnv.__init__():
	1.	Create self.sensor_interface = SensorInterface().
	2.	Spawn ego vehicle via CARLA API.
	3.	Spawn each sensor with Sensor_manager.spawn(name, attributes, self.sensor_interface, self.ego_vehicle).
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


	•	IMU Sensor (optional)

name: "imu_sensor"
attributes:
  type: "sensor.other.imu"
  transform: "0,0,0,0,0,0"


	•	GNSS Sensor

name: "gnss_sensor"
attributes:
  type: "sensor.other.gnss"
  transform: "0,0,2,0,0,0"


	•	Semantic Segmentation Camera (optional)

name: "sem_seg_camera"
attributes:
  type: Config.SEMANTIC_CAMERA               # "sensor.camera.semantic_segmentation"
  transform: "1.5,0,2.4,0,0,0"
  image_size_x: str(Config.IMAGE_WIDTH)
  image_size_y: str(Config.IMAGE_HEIGHT)
  fov: str(Config.CAMERA_FOV)



4.3 Pulling Sensor Data

In CarlaEnv.step(action), after world.tick():

sensor_data = self.sensor_interface.get_data()

	•	Event-based sensors (Collision, LaneInvasion) appear in sensor_data only if an event occurred since the last pull.
	•	Streaming sensors (IMU, GNSS, semantic camera) each provide exactly one reading per pull.

⸻

5. High-Level Objectives
	1.	Primary Objective: Navigate from a start location to a goal location (carla.Location).
	•	Goal reached if ego_vehicle.get_transform().location.distance(goal_location) < 2 m.
	2.	Secondary Objectives:
	•	Avoid Collisions (any actor).
	•	Maintain Lane Discipline (stay near lane center, avoid crossing).
	•	Regulate Speed: keep close to Config.TARGET_SPEED, never below MIN_SPEED or above MAX_SPEED.
	•	Minimize Lateral Drift (stay within MAX_DISTANCE_FROM_CENTER).
	•	Efficient Progress (finish quickly, minimize steps).
	3.	Optional Objectives:
	•	Pedestrian Yielding (detect via semantic camera or semantic lidar).
	•	Traffic Signal Compliance (detect red lights via semantic segmentation or map waypoints).
	•	Smoothness (penalize high jerk from IMU data).

⸻

6. Reward Components

6.1 Collision Detection
	•	Sensor: collision_sensor (event‐based).
	•	When triggered: apply flat penalty Config.COLLISION_REWARD (–100) and terminate the episode.

6.2 Lane-Keeping & Deviation
	•	Sensor: lane_invasion_sensor (event‐based) + CARLA map API.
	•	If lane_invasion_sensor appears:
	•	Penalty: Config.LANE_DEVIATION_REWARD (–50).
	•	Else: calculate lateral distance to lane center:

waypoint = world.get_map().get_waypoint(current_loc, project_to_road=True)
center_loc = waypoint.transform.location
dist_center = current_loc.distance(center_loc)

	•	If dist_center <= Config.MAX_DISTANCE_FROM_CENTER (3 m):
	•	Bonus: Config.MAX_DISTANCE_FROM_CENTER - dist_center (∈ [0, 3]).
	•	Else:
	•	Penalty: Config.LANE_DEVIATION_REWARD (–50).

6.3 Speed Regulation
	•	Sensor: None (use ego_vehicle.get_velocity()).
	•	Compute: current_speed = √(v.x² + v.y² + v.z²) (m/s).
	•	Penalties & Bonuses:
	1.	Below MIN_SPEED (15 m/s):
	•	Penalty: Config.SLOW_SPEED_REWARD (–50).
	2.	Above MAX_SPEED (25 m/s):
	•	Penalty: Config.SPEED_PENALTY (–1 per timestep).
	3.	Proximity to TARGET_SPEED (22 m/s):
	•	speed_error = |current_speed – TARGET_SPEED|.
	•	Bonus: max(0, 10 – speed_error) (∈ [0, 10]).

6.4 Distance to Lane Center
	•	Sensor: None (use map API as in Section 6.2).
	•	Reward/Penalty: identical to lane-keeping bonus/penalty in Section 6.2 (lateral distance).

6.5 Progress Toward Goal
	•	Sensor: None (use ego_vehicle.get_transform().location). GNSS can supplement, but direct location is simpler.
	•	On first step:

prev_dist = current_loc.distance(goal_location)
init_dist = prev_dist


	•	Each step:

cur_dist = current_loc.distance(goal_location)
delta_d = prev_dist – cur_dist
if delta_d > 0:
    progress_reward = (delta_d / init_dist) * 100.0  # normalized to [0, 100]
    reward += progress_reward
prev_dist = cur_dist


	•	If cur_dist < 2.0 m:
	•	Bonus: +100 (completion) and terminate.

6.6 Time/Efficiency
	•	Sensor: None (use internal self.elapsed_steps).
	•	Per-step penalty:

reward += Config.SPEED_PENALTY  # –1 each step


	•	If self.elapsed_steps >= Config.MAX_TIMESTEPS:
	•	Penalty: –100, terminate due to timeout.

⸻

7. Reward Summary Tables

Positive Rewards

Event / Condition	Reward	Description
Reaching Goal	+100	Within 2 m of goal; ends episode.
Progress Toward Goal	+ (delta_d / init_dist) * 100	Proportional to distance reduction each step.
Lane-Keeping (dist ≤ 3 m)	+ (3 – dist_center) [0 to +3]	More bonus when closer to center.
Target-Speed Proximity	`+ max(0, 10 –	speed – 22

Negative Rewards

Event / Condition	Reward	Description
Collision	–100	Any collision detected; ends episode.
Lane Deviation (crossing lane)	–50	Crossing any lane marking or off-road.
Speed < MIN_SPEED (15 m/s)	–50	Driving too slowly (unless required).
Speed > MAX_SPEED (25 m/s)	–1 per step	Over-speeding penalty each timestep.
Distance from center > 3 m	–50	Off-road penalty (treated same as lane crossing).
Timeout (steps ≥ 7500)	–100	Episode ends due to exceeding max timesteps.


⸻

8. Integration Notes
	•	Sensor Retrieval: In CarlaEnv.step(), call:

sensor_data = self.sensor_interface.get_data()

to access parsed outputs:
	•	"collision_sensor": [other_actor, impulse] if a collision occurred.
	•	"lane_invasion_sensor": [transform, crossed_lane_markings] if lane crossing occurred.
	•	"imu_sensor": [accX, accY, accZ, gyroX, gyroY, gyroZ, compass] each step (optional).
	•	"gnss_sensor": [lat, lon, alt] each step (optional).
	•	"sem_seg_camera": image array each step (optional).

	•	Order of Checks in step():
	1.	Advance CARLA (world.tick()).
	2.	Retrieve sensor_data.
	3.	Collision → apply –100 and terminate.
	4.	Lane invasion → apply –50; else compute lane-keeping bonus (up to +3).
	5.	Compute speed penalties/bonuses (–50 if < 15 m/s; –1 if > 25 m/s; + up to +10 for |speed–22|).
	6.	Compute progress reward (normalized × 100).
	7.	If goal reached (dist < 2 m) → +100 and terminate.
	8.	Time penalty (–1 each step); if ≥ 7500 steps → –100 and terminate.
	9.	Return (observation, reward, done, info).
	•	Observation Design (example):
	•	[current_speed, distance_to_goal, distance_to_center] as a simple 3D vector.
	•	Optionally include semantic camera or GNSS data if needed for other tasks.
	•	Episode Reset:
	•	Destroy sensors and ego vehicle, respawn both, reinitialize counters (elapsed_steps=0, prev_dist=None, init_dist=None).

⸻

9. References
	1.	CARLA Simulator Documentation (Python API):
https://carla.readthedocs.io/en/latest/python_api/
	2.	Gymnasium API Reference:
https://gymnasium.farama.org/
	3.	CARLA Sensor Tutorials:
	•	https://carla.readthedocs.io/en/latest/tuto_sensors/
	•	https://carla.readthedocs.io/en/latest/tuto_Gym_env/
	4.	Global Route Planning (optional):
	•	agents/navigation/global_route_planner.py
	•	agents/navigation/global_route_planner_dao.py


