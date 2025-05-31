# Reward Engineering for CARLA Simulator + Gymnasium

**Author:** Alexander Assal  
**Date:** May 31, 2025  

---

## 🧠 Goal

Design a reward structure for a Gymnasium-compatible RL agent in CARLA that encourages:
1. Safe, collision-free driving  
2. Compliance with traffic rules (lane keeping, speed limits, pedestrian and traffic-light respect)  
3. Efficient progress toward a predefined goal location  

Sensors used:
- **Collision** (`sensor.other.collision`)  
- **Lane invasion** (`sensor.other.lane_invasion`)  
- **Vehicle velocity** via `ego_vehicle.get_velocity()`  
- **Map/waypoint queries** via `world.get_map().get_waypoint(location)`  
- **IMU** (`sensor.other.imu`) for smoothness/jerk  
- **GNSS** (`sensor.other.gnss`) for approximate location  
- **Semantic segmentation camera** (`sensor.camera.semantic_segmentation`) for detecting pedestrians and traffic lights  

---

## 🛣️ Directory & File Structure

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

## ⚙️ Configuration Parameters (`Config.py`)

All constants below appear exactly in `Config.py`. “`Config.XYZ`” refers to these names directly.

```python
# Connection & Episode
HOST = "localhost"
PORT = 2000
TIMEOUT = 30.0
EPISODE_LENGTH = 120        # seconds per episode
MAX_TIMESTEPS = 7500        # steps per episode

# Vehicle & Traffic
CAR_NAME = 'vehicle.mini.cooper'
NUMBER_OF_VEHICLES = 10
NUMBER_OF_PEDESTRIANS = 30

# Action Space
CONTINUOUS_ACTION = True    # True = continuous [steer, throttle, brake]; False = discrete

# Camera (for pedestrian/traffic-light detection)
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
LANE_DEVIATION_REWARD = -50        # flat penalty for crossing lane
SLOW_SPEED_REWARD = -50            # penalty for speed < MIN_SPEED
SPEED_PENALTY = -1                 # per-step penalty if speed > MAX_SPEED

PEDESTRIAN_YIELD_REWARD = +10      # reward for yielding to pedestrian in crosswalk
PEDESTRIAN_VIOLATION_PENALTY = -30 # penalty for passing close to pedestrian without stopping
TRAFFIC_LIGHT_STOP_REWARD = +5     # reward for stopping at red light properly
TRAFFIC_LIGHT_RUN_PENALTY = -20    # penalty for running a red light


⸻

🚦 Sensor Setup & Data Retrieval

1. Sensors Used
	•	Collision (sensor.other.collision)
	•	Event-based: triggers on any collision
	•	Parsed data: [other_actor, impulse_value]
	•	LaneInvasion (sensor.other.lane_invasion)
	•	Event-based: triggers when crossing any lane marking
	•	Parsed data: [transform, crossed_lane_markings]
	•	Imu (sensor.other.imu)
	•	Streaming: returns [accX, accY, accZ, gyroX, gyroY, gyroZ, compass]
	•	Used for smoothness/jerk calculation
	•	Gnss (sensor.other.gnss)
	•	Streaming: returns [latitude, longitude, altitude]
	•	Approximate global position; mostly use ego_vehicle.get_transform().location
	•	CameraSemanticSegmentation (sensor.camera.semantic_segmentation)
	•	Streaming: returns H×W×3 uint8 image
	•	Used to detect pedestrian pixels and traffic-light pixels

2. Spawning & Registering

Inside CarlaEnv.__init__():
	1.	Instantiate self.sensor_interface = SensorInterface().
	2.	Spawn ego vehicle with CARLA API.
	3.	Spawn each sensor with Sensor_manager.spawn(name, attributes, self.sensor_interface, self.ego_vehicle):
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
  type: Config.SEMANTIC_CAMERA               # "sensor.camera.semantic_segmentation"
  transform: "1.5,0,2.4,0,0,0"
  image_size_x: str(Config.IMAGE_WIDTH)
  image_size_y: str(Config.IMAGE_HEIGHT)
  fov: str(Config.CAMERA_FOV)



3. Pulling Sensor Data

In CarlaEnv.step(action), after world.tick():

sensor_data = self.sensor_interface.get_data()

	•	Event-based sensors appear only if an event occurred since the last call (e.g., "collision_sensor", "lane_invasion_sensor").
	•	Streaming sensors (IMU, GNSS, semantic camera) provide one reading each timestep.

⸻

🎯 High-Level Objectives
	1.	Primary Objective: Drive from a start location to a predefined goal location (carla.Location).
	•	Goal reached if ego_vehicle.get_transform().location.distance(goal_location) < 2 m.
	2.	Secondary Objectives:
	•	Avoid collisions with any actor.
	•	Maintain lane discipline (stay near lane center, avoid crossing lane markings).
	•	Regulate speed: remain near TARGET_SPEED, avoid below MIN_SPEED or above MAX_SPEED.
	•	Minimize lateral drift (maintain dist_center ≤ MAX_DISTANCE_FROM_CENTER).
	•	Efficient progress (minimize time/steps).
	•	Respect pedestrians at crosswalks.
	•	Obey traffic lights (stop at red).
	3.	Tertiary Objectives (Trade-Offs):
	•	Smoothness: penalize high jerk (via IMU) and abrupt steering.

⸻

🔢 Reward Components

1. Collision Detection
	•	Sensor: "collision_sensor" from sensor.other.collision.
	•	When triggered: apply –100 (Config.COLLISION_REWARD), set done = True, return immediately.

2. Lane-Keeping & Deviation
	•	Sensor: "lane_invasion_sensor" from sensor.other.lane_invasion + map API.
	•	If "lane_invasion_sensor" in sensor_data:
	•	Penalty: –50 (Config.LANE_DEVIATION_REWARD).
	•	Else:
	•	Compute dist_center via:

waypoint = world.get_map().get_waypoint(current_loc, project_to_road=True)
center_loc = waypoint.transform.location
dist_center = current_loc.distance(center_loc)


	•	If dist_center ≤ Config.MAX_DISTANCE_FROM_CENTER (3 m):
	•	Bonus: Config.MAX_DISTANCE_FROM_CENTER – dist_center (∈ [0, 3]).
	•	If dist_center > Config.MAX_DISTANCE_FROM_CENTER:
	•	Penalty: –50 (Config.LANE_DEVIATION_REWARD).

3. Speed Regulation
	•	Sensor: None (use ego_vehicle.get_velocity()).
	•	Compute: current_speed = √(v.x² + v.y² + v.z²) (m/s).
	•	Penalties & Bonuses:
	1.	If current_speed < Config.MIN_SPEED (15 m/s):
	•	Penalty: –50 (Config.SLOW_SPEED_REWARD).
	2.	If current_speed > Config.MAX_SPEED (25 m/s):
	•	Penalty: –1 per timestep (Config.SPEED_PENALTY).
	3.	Proximity to TARGET_SPEED (22 m/s):
	•	speed_error = |current_speed – Config.TARGET_SPEED|.
	•	Bonus: max(0, 10 – speed_error) (∈ [0, 10]).

4. Distance to Lane Center
	•	Sensor: None (map API, same as lane-keeping).
	•	Reward/Penalty: identical to lane-keeping logic above (Section 2).

5. Progress Toward Goal
	•	Sensor: None (use ego_vehicle.get_transform().location), GNSS can supplement.
	•	On first step:

prev_dist = current_loc.distance(goal_location)
init_dist = prev_dist


	•	Each step:

cur_dist = current_loc.distance(goal_location)
delta_d = prev_dist – cur_dist
if delta_d > 0:
    progress_reward = (delta_d / init_dist) * 100.0
    reward += progress_reward
prev_dist = cur_dist


	•	If cur_dist < 2 m:
	•	Bonus: +100 (Config.EPISODE_LENGTH?), set done = True.

6. Time/Efficiency
	•	Sensor: None (internal self.elapsed_steps).
	•	Per-step penalty: –1 (Config.SPEED_PENALTY).
	•	If self.elapsed_steps ≥ Config.MAX_TIMESTEPS:
	•	Penalty: –100, set done = True.

7. Pedestrian Yielding
	•	Sensor: "sem_seg_camera" from sensor.camera.semantic_segmentation or "semantic_lidar".
	•	If a pedestrian is present in the crosswalk near the ego vehicle (within ~5 m) and current_speed < 0.5 m/s:
	•	Reward: +10 (Config.PEDESTRIAN_YIELD_REWARD).
	•	If ego vehicle passes within 1.5 m of a pedestrian in crosswalk at current_speed ≥ 0.5 m/s:
	•	Penalty: –30 (Config.PEDESTRIAN_VIOLATION_PENALTY).

8. Traffic-Light Compliance
	•	Sensor: "sem_seg_camera" (semantic segmentation) to detect red/green light pixels, plus map-based stop-line distance check.
	•	If ego vehicle stops (speed < 0.5 m/s) before the stop line at a red light for ≥1 s:
	•	Reward: +5 (Config.TRAFFIC_LIGHT_STOP_REWARD).
	•	If ego vehicle crosses the stop line at a red light with current_speed > 1 m/s:
	•	Penalty: –20 (Config.TRAFFIC_LIGHT_RUN_PENALTY).

⸻

📊 Reward Summary Tables

Positive Rewards

Condition	Reward	Description
Reaching Goal	+100	Within 2 m of goal; episode ends.
Progress Toward Goal	+ (Δd / initial_distance) × 100	Proportional to distance reduction each step.
Lane-Keeping (dist_center ≤ 3 m)	+ (3 – dist_center) (0 to +3)	More bonus when closer to lane center.
Target-Speed Proximity (	speed – 22	)
Yielding to Pedestrian in Crosswalk	+10	Stopping when pedestrian present in crosswalk.
Stopping Properly at Red Light (≥1 s before stop line)	+5	Comes to full stop (speed < 0.5) at red light.

Negative Rewards

Condition	Reward	Description
Collision	–100	Any collision detected; episode ends.
Lane Crossing / Off-Road (dist_center > 3 m)	–50	Crossing lane marking or drifting off lane.
Speed < MIN_SPEED (15 m/s)	–50	Driving too slowly (unless required).
Speed > MAX_SPEED (25 m/s)	–1 per step	Over-speeding penalty each timestep.
Passing Pedestrian Without Yield (≤1.5 m)	–30	Passing too close to pedestrian without stopping.
Running Red Light	–20	Crossing stop line at red light with speed > 1 m/s.
Timeout (elapsed_steps ≥ 7500)	–100	Episode ends due to exceeding max timesteps.


⸻

📝 Integration Notes
	1.	Pulling Sensor Data:
	•	Call sensor_data = self.sensor_interface.get_data() each step().
	•	Check presence of "collision_sensor", "lane_invasion_sensor" for events.
	•	Retrieve "imu_sensor", "gnss_sensor", "sem_seg_camera" values as needed.
	2.	Reward Calculation Order (in step()):
	1.	Collision → –100, done=True, return.
	2.	Lane Crossing → –50 or lane-keeping bonus (up to +3).
	3.	Speed Regulation → –50 if < 15 m/s; –1 if > 25 m/s; + up to +10 for closeness to 22 m/s.
	4.	Distance to Lane Center → same as lane-keeping above.
	5.	Progress Toward Goal → normalized ×100; if within 2 m → +100, done=True.
	6.	Pedestrian Yielding → +10 if stopping when pedestrian in crosswalk; –30 if passing too close at ≥0.5 m/s.
	7.	Traffic-Light Compliance → +5 if correctly stopped at red; –20 if running red.
	8.	Time Penalty → –1 per timestep; if elapsed_steps ≥ MAX_TIMESTEPS → –100, done=True.
	9.	Return (observation, reward, done, info).
	3.	Observation Design Example:

[ current_speed, 
  distance_to_goal, 
  distance_to_center ]

	•	Extend with semantic segmentation (sem_seg_camera) or GNSS as needed.

	4.	Episode Reset:
	•	Destroy sensors: self.sensor_interface.destroy().
	•	Destroy ego vehicle: self.ego_vehicle.destroy().
	•	Respawn vehicle and sensors.
	•	Reset: self.elapsed_steps = 0, prev_dist = None, init_dist = None.
	5.	Tuning Suggestions:
	•	Increase +100 goal reward if agent struggles to reach it.
	•	Multiply lane-keeping bonus (3 – dist_center) by a factor (e.g., 5) if too weak.
	•	Increase time penalty to –2 per step if agent loiters.

⸻

📚 References
	1.	CARLA Python API Documentation:
https://carla.readthedocs.io/en/latest/python_api/
	2.	Gymnasium API:
https://gymnasium.farama.org/
	3.	CARLA Sensor Tutorials:
	•	https://carla.readthedocs.io/en/latest/tuto_sensors/
	•	https://carla.readthedocs.io/en/latest/tuto_Gym_env/
	4.	Global Route Planning (if needed):
	•	agents/navigation/global_route_planner.py
	•	agents/navigation/global_route_planner_dao.py

