# Reward Engineering for CARLA Simulator

**Author:** [Author Name]  
**Date:** [YYYY-MM-DD]  

---

## Table of Contents

1. [Overview](#overview)  
2. [Directory & File Structure](#directory--file-structure)  
3. [Configuration Parameters (`Config.py`)](#configuration-parameters-configpy)  
4. [Sensor Infrastructure](#sensor-infrastructure)  
   1. [Sensor Interface (`Sensor_interface.py`)](#sensor-interface-sensor_interfacepy)  
   2. [Sensor Manager (`Sensor_manager.py`)](#sensor-manager-sensor_managerpy)  
   3. [Supported Sensors & Attributes](#supported-sensors--attributes)  
   4. [Spawning & Registering Sensors in `CarlaEnv`](#spawning--registering-sensors-in-carlaenv)  
   5. [Pulling Sensor Data Each Timestep](#pulling-sensor-data-each-timestep)  
5. [High-Level Objectives & Terminology](#high-level-objectives--terminology)  
6. [Reward Components & Sensor Dependencies](#reward-components--sensor-dependencies)  
   1. [Collision Detection](#collision-detection)  
   2. [Lane Deviation / Lane-Keeping](#lane-deviation--lane-keeping)  
   3. [Speed Regulation](#speed-regulation)  
   4. [Distance from Road Center](#distance-from-road-center)  
   5. [Progress Toward Goal](#progress-toward-goal)  
   6. [Time/Efficiency Penalties](#timeefficiency-penalties)  
7. [Detailed Reward Calculation & Pseudocode](#detailed-reward-calculation--pseudocode)  
   1. [`step()` Workflow Overview](#step-workflow-overview)  
   2. [Initialization & Sensor Data Pull](#initialization--sensor-data-pull)  
   3. [Immediate Collision Penalty](#immediate-collision-penalty)  
   4. [Lane Deviation Penalty & Lane-Keeping Bonus](#lane-deviation-penalty--lane-keeping-bonus)  
   5. [Speed Regulation Reward/Penalty](#speed-regulation-rewardpenalty)  
   6. [Distance from Center Reward/Penalty](#distance-from-center-rewardpenalty)  
   7. [Progress Toward Goal](#progress-toward-goal-1)  
   8. [Time/Efficiency Penalty](#timeefficiency-penalty)  
   9. [Episode Termination Conditions](#episode-termination-conditions)  
8. [Sensor Initialization Example in `CarlaEnv`](#sensor-initialization-example-in-carlaenv)  
9. [Tuning & Scaling Guidelines](#tuning--scaling-guidelines)  
10. [Appendix: Helper Methods & CARLA API Snippets](#appendix-helper-methods--carla-api-snippets)  
11. [References](#references)  

---

## 1. Overview

A comprehensive reward engineering scheme for a Gymnasium-compatible reinforcement learning environment in CARLA is described. Components include:

- **Sensor.py**: Base classes (`BaseSensor`, `CarlaSensor`) and specific sensors (`Collision`, `LaneInvasion`, `Imu`, `Gnss`, etc.).  
- **Sensor_interface.py**: Buffers sensor data (continuous and event-based) into queues.  
- **Sensor_manager.py**: Spawns CARLA sensors via a unified `spawn(name, attributes, interface, parent)` API.  
- **Config.py**: Contains simulation and reward constants (speeds, penalties, episode lengths).

A class named `CarlaEnv` is assumed to implement `__init__()`, `reset()`, `step(action)`, and `close()`.

---

## 2. Directory & File Structure

/project_root/
│
├── envs/
│   ├── init.py
│   └── carla_env.py            # Gymnasium environment wrapper (CarlaEnv)
│
├── sensors/
│   ├── Sensor.py               # BaseSensor, CarlaSensor, and all sensor classes
│   ├── Sensor_interface.py     # Data buffering and event sensor management
│   └── Sensor_manager.py       # Helper to spawn specific sensors
│
├── Config.py                   # Simulation & reward parameters
├── reward_engineering.md       # ← (This file)
├── Traffic_config.py           # Traffic-related spawn parameters
├── Traffic_manager.py          # Spawns NPC traffic (vehicles & walkers)
├── Connection_manager.py       # CARLA client connection manager
└── requirements.txt            # Dependencies: carla, gymnasium, numpy, etc.

---

## 3. Configuration Parameters (`Config.py`)

```python
# Connection settings
HOST = "localhost"
PORT = 2000
TIMEOUT = 30.0

# Vehicle settings
CAR_NAME = 'vehicle.mini.cooper'
EPISODE_LENGTH = 120           # seconds per episode
MAX_TIMESTEPS = 7500           # max simulation steps per episode

# Traffic settings
NUMBER_OF_VEHICLES = 10
NUMBER_OF_PEDESTRIANS = 30

# Action space settings
CONTINUOUS_ACTION = True       # True: (steer, throttle, brake); False: discrete actions

# Display settings
VISUAL_DISPLAY = True

# Camera settings (optional)
RGB_CAMERA = 'sensor.camera.rgb'
SEMANTIC_CAMERA = 'sensor.camera.semantic_segmentation'
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 80
CAMERA_FOV = 125

# Reward parameters
TARGET_SPEED = 22.0                # target speed (m/s)
MAX_SPEED = 25.0                   # upper bound speed (m/s)
MIN_SPEED = 15.0                   # lower bound speed (m/s)
MAX_DISTANCE_FROM_CENTER = 3.0     # lane center threshold (meters)
COLLISION_REWARD = -100            # flat collision penalty
LANE_DEVIATION_REWARD = -50        # lane crossing penalty
SLOW_SPEED_REWARD = -50            # too slow penalty (below MIN_SPEED)
SPEED_PENALTY = -1                 # per-timestep penalty if > MAX_SPEED


⸻

4. Sensor Infrastructure

4.1 Sensor Interface (Sensor_interface.py)
	•	Purpose: Manages registration of sensors and buffers incoming data in two queues:
	•	_data_buffers for streaming sensors (e.g., cameras, IMU, GNSS).
	•	_event_data_buffers for event-based sensors (e.g., collisions, lane invasions).
	•	Key Methods:

class SensorInterface(object):
    def __init__(self):
        self._sensors = {}              # name → sensor instance (streaming)
        self._event_sensors = {}        # name → sensor instance (event-based)
        self._data_buffers = queue.Queue()
        self._event_data_buffers = queue.Queue()
        self._queue_timeout = 10

    def register(self, name, sensor):
        if sensor.is_event_sensor():
            self._event_sensors[name] = sensor
        else:
            self._sensors[name] = sensor

    def get_data(self):
        """
        Blocks until one reading from each streaming sensor is received, then collects
        zero-or-more event sensor readings. Returns a dict:
        {sensor_name: (frame, parsed_data)}.
        """
        data_dict = {}
        try:
            while len(data_dict) < len(self._sensors):
                sensor_name, frame, parsed = self._data_buffers.get(True, self._queue_timeout)
                data_dict[sensor_name] = (frame, parsed)
        except queue.Empty:
            raise RuntimeError("A sensor took too long to send data")

        for name in self._event_sensors:
            try:
                sensor_name, frame, parsed = self._event_data_buffers.get_nowait()
                data_dict[sensor_name] = (frame, parsed)
            except queue.Empty:
                pass

        return data_dict

    def destroy(self):
        for sensor in {**self._sensors, **self._event_sensors}.values():
            sensor.destroy()
        self._data_buffers = queue.Queue()
        self._event_data_buffers = queue.Queue()



4.2 Sensor Manager (Sensor_manager.py)
	•	Purpose: Spawns specific CARLA sensors via a single spawn(name, attributes, interface, parent) method.
	•	Key Method:

class SensorManager(object):
    @staticmethod
    def spawn(name, attributes, interface, parent):
        type_ = attributes.get("type", "")
        if type_ == "sensor.camera.rgb":
            return CameraRGB(name, attributes, interface, parent)
        elif type_ == "sensor.camera.semantic_segmentation":
            return CameraSemanticSegmentation(name, attributes, interface, parent)
        elif type_ == "sensor.camera.depth":
            return CameraDepth(name, attributes, interface, parent)
        elif type_ == "sensor.camera.dvs":
            return CameraDVS(name, attributes, interface, parent)
        elif type_ == "sensor.lidar.ray_cast":
            return Lidar(name, attributes, interface, parent)
        elif type_ == "sensor.lidar.ray_cast_semantic":
            return SemanticLidar(name, attributes, interface, parent)
        elif type_ == "sensor.other.radar":
            return Radar(name, attributes, interface, parent)
        elif type_ == "sensor.other.gnss":
            return Gnss(name, attributes, interface, parent)
        elif type_ == "sensor.other.imu":
            return Imu(name, attributes, interface, parent)
        elif type_ == "sensor.other.lane_invasion":
            return LaneInvasion(name, attributes, interface, parent)
        elif type_ == "sensor.other.collision":
            return Collision(name, attributes, interface, parent)
        elif type_ == "sensor.other.obstacle":
            return Obstacle(name, attributes, interface, parent)
        else:
            raise RuntimeError(f"Sensor type {type_} not supported.")


	•	Example Attributes Dictionary:

cam_attrs = {
    "type": Config.RGB_CAMERA,             # "sensor.camera.rgb"
    "transform": "0.8,0,1.7,0,0,0",         # x, y, z, roll, pitch, yaw
    "image_size_x": str(Config.IMAGE_WIDTH),
    "image_size_y": str(Config.IMAGE_HEIGHT),
    "fov": str(Config.CAMERA_FOV)
}
sensor = SensorManager.spawn("rgb_front", cam_attrs, sensor_interface, ego_vehicle)



4.3 Supported Sensors & Attributes

Sensor Class	Type String	Event-Based?	Parsed Data Format	Usage
Collision	sensor.other.collision	Yes	[other_actor, impulse_value]	Detect collisions for penalty
LaneInvasion	sensor.other.lane_invasion	Yes	[transform, crossed_lane_markings]	Detect lane crossings
Imu	sensor.other.imu	No	[accX, accY, accZ, gyroX, gyroY, gyroZ, comp]	Compute acceleration & jerk (optional)
Gnss	sensor.other.gnss	No	[latitude, longitude, altitude]	Estimate position & distance to goal
CameraSemanticSegmentation	sensor.camera.semantic_segmentation	No	H×W×3 uint8 array (class IDs colored)	Detect pedestrians/traffic lights (optional)
SemanticLidar	sensor.lidar.ray_cast_semantic	No	N×6 array [x, y, z, cos(angle), actor_index, tag]	Identify nearby actors (optional)
Lidar	sensor.lidar.ray_cast	No	M×4 array [x, y, z, intensity]	Obstacle detection (optional)
Radar	sensor.other.radar	No	P×4 array [depth, azimuth, altitude, velocity]	Relative velocity detection (optional)

Note: Minimal reward scheme requires only Collision and LaneInvasion. Speed can be obtained via ego_vehicle.get_velocity(), and position via ego_vehicle.get_transform().location.

4.4 Spawning & Registering Sensors in CarlaEnv

Within CarlaEnv.__init__():
	1.	Connect to CARLA:

from Connection_manager import ClientConnection
conn = ClientConnection(town="Town03")
client, world = conn.connect(host=Config.HOST, port=Config.PORT, timeout=Config.TIMEOUT)


	2.	Instantiate SensorInterface:

from sensors.Sensor_interface import SensorInterface
self.sensor_interface = SensorInterface()


	3.	Spawn Ego Vehicle:

blueprint = world.get_blueprint_library().find(Config.CAR_NAME)
spawn_point = random.choice(world.get_map().get_spawn_points())
self.ego_vehicle = world.spawn_actor(blueprint, spawn_point)


	4.	Spawn Required Sensors via SensorManager:

from sensors.Sensor_manager import SensorManager

# Collision Sensor
collision_attrs = {"type": "sensor.other.collision", "transform": "0,0,0,0,0,0"}
SensorManager.spawn("collision_sensor", collision_attrs, self.sensor_interface, self.ego_vehicle)

# Lane Invasion Sensor
lane_invasion_attrs = {"type": "sensor.other.lane_invasion", "transform": "0,0,0,0,0,0"}
SensorManager.spawn("lane_invasion_sensor", lane_invasion_attrs, self.sensor_interface, self.ego_vehicle)

# IMU Sensor (optional)
imu_attrs = {"type": "sensor.other.imu", "transform": "0,0,0,0,0,0"}
SensorManager.spawn("imu_sensor", imu_attrs, self.sensor_interface, self.ego_vehicle)

# GNSS Sensor (for distance-to-goal)
gnss_attrs = {"type": "sensor.other.gnss", "transform": "0,0,2,0,0,0"}
SensorManager.spawn("gnss_sensor", gnss_attrs, self.sensor_interface, self.ego_vehicle)

# Semantic Segmentation Camera (optional)
sem_cam_attrs = {
    "type": Config.SEMANTIC_CAMERA,
    "transform": "1.5,0,2.4,0,0,0",
    "image_size_x": str(Config.IMAGE_WIDTH),
    "image_size_y": str(Config.IMAGE_HEIGHT),
    "fov": str(Config.CAMERA_FOV)
}
SensorManager.spawn("sem_seg_camera", sem_cam_attrs, self.sensor_interface, self.ego_vehicle)


	5.	Initialize Internal Tracking Variables:

self.elapsed_steps = 0
start_loc = self.ego_vehicle.get_transform().location
self.goal_location = self._choose_random_goal(start_loc)
self.previous_distance_to_goal = None
self.initial_distance_to_goal = None
self.previous_speed = 0.0
self.lane_invasion_count = 0



4.5 Pulling Sensor Data Each Timestep

In CarlaEnv.step(action):

# After world.tick() and before computing rewards:
sensor_data = self.sensor_interface.get_data()
# Example keys (if event occurred): "collision_sensor", "lane_invasion_sensor"
# Example keys (always present): "imu_sensor", "gnss_sensor", "sem_seg_camera"

	•	Event-based sensors appear only when an event has occurred since the last call.
	•	Streaming sensors return one entry per call.

⸻

5. High-Level Objectives & Terminology
	1.	Primary Objective: Reach a Predefined Goal Location
	•	A fixed goal_location (carla.Location) is selected.
	•	When ego_vehicle.get_transform().location.distance(goal_location) < 2 m, the goal is reached.
	2.	Secondary Objectives:
	1.	Avoid collisions with any actor.
	2.	Maintain lane discipline (stay near lane center, avoid crossing lane markings).
	3.	Regulate speed: remain near TARGET_SPEED, avoid going below MIN_SPEED or above MAX_SPEED.
	4.	Minimize distance from lane center.
	5.	Make efficient progress (minimize time/steps).
	3.	Tertiary (Optional) Objectives:
	•	Pedestrian yielding (use semantic segmentation or semantic lidar).
	•	Traffic signal compliance (detect via semantic segmentation or map waypoints).
	•	Smoothness (penalize high jerk or rapid steering).

⸻

6. Reward Components & Sensor Dependencies

6.1 Collision Detection
	•	Dependency: collision_sensor (event-based).
	•	Parsed Data: [other_actor, impulse_value].
	•	Reward: Config.COLLISION_REWARD (–100), then episode terminates.

6.2 Lane Deviation / Lane-Keeping
	•	Dependency: lane_invasion_sensor (event-based) and CARLA map API.
	•	Parsed Data (event): [transform, crossed_lane_markings].
	•	Reward/Penalty:
	•	If lane_invasion_sensor present: add Config.LANE_DEVIATION_REWARD (–50).
	•	Otherwise, compute dist_center = lateral distance to lane center:

waypoint = world.get_map().get_waypoint(current_loc, project_to_road=True)
center_loc = waypoint.transform.location
dist_center = current_loc.distance(center_loc)

	•	If dist_center <= Config.MAX_DISTANCE_FROM_CENTER (3.0 m):

center_bonus = Config.MAX_DISTANCE_FROM_CENTER - dist_center  # ∈ [0, +3]
reward += center_bonus


	•	If dist_center > Config.MAX_DISTANCE_FROM_CENTER:

reward += Config.LANE_DEVIATION_REWARD  # –50



6.3 Speed Regulation
	•	Dependency: Direct CARLA API: ego_vehicle.get_velocity().
	•	Computed Value: current_speed = √(v.x² + v.y² + v.z²) in m/s.
	•	Reward/Penalty:
	1.	If current_speed < Config.MIN_SPEED (15.0 m/s):

reward += Config.SLOW_SPEED_REWARD  # –50


	2.	If current_speed > Config.MAX_SPEED (25.0 m/s):

reward += Config.SPEED_PENALTY  # –1 per timestep


	3.	Target-speed proximity bonus:

speed_error = abs(current_speed - Config.TARGET_SPEED)  # 22.0 m/s
speed_bonus = max(0, 10 - speed_error)  # ∈ [0, +10]
reward += speed_bonus



6.4 Distance from Road Center
	•	Dependency: CARLA map API (same as lane center calculation).
	•	Reward/Penalty:
	•	If dist_center <= Config.MAX_DISTANCE_FROM_CENTER:

center_bonus = Config.MAX_DISTANCE_FROM_CENTER - dist_center  # ∈ [0, +3]
reward += center_bonus


	•	Else:

reward += Config.LANE_DEVIATION_REWARD  # –50



6.5 Progress Toward Goal
	•	Dependencies:
	•	gnss_sensor (parsed as [latitude, longitude, altitude], but easier to use ego_vehicle.get_transform().location).
	•	CARLA map API for direct Euclidean distance to goal_location.
	•	Reward:
	1.	On first timestep, initialize:

self.previous_distance_to_goal = current_loc.distance(self.goal_location)
self.initial_distance_to_goal = self.previous_distance_to_goal


	2.	Every subsequent timestep:

distance_to_goal = current_loc.distance(self.goal_location)
delta_d = self.previous_distance_to_goal - distance_to_goal
if delta_d > 0:
    progress_reward = (delta_d / self.initial_distance_to_goal) * 100.0
    reward += progress_reward
self.previous_distance_to_goal = distance_to_goal


	3.	If distance_to_goal < 2.0 m:

reward += +100  # large completion bonus
done = True
info["reached_goal"] = True



6.6 Time/Efficiency Penalties
	•	Dependency: Internal counter self.elapsed_steps.
	•	Reward/Penalty:
	1.	Per-timestep penalty:

reward += Config.SPEED_PENALTY  # –1 each step


	2.	Timeout check:

if self.elapsed_steps >= Config.MAX_TIMESTEPS:
    reward += -100
    done = True
    info["timeout"] = True



⸻

7. Detailed Reward Calculation & Pseudocode

7.1 step() Workflow Overview
	1.	Apply control based on action (continuous or discrete).
	2.	Advance simulator (synchronous tick).
	3.	Retrieve sensor data via sensor_interface.get_data().
	4.	Compute current speed and location using CARLA API.
	5.	Initialize reward = 0.0 and info dictionary.
	6.	Check for collision event and apply penalty.
	7.	Check for lane invasion event or compute lane-keeping bonus.
	8.	Apply speed regulation reward/penalty.
	9.	Compute distance-to-center reward/penalty.
	10.	Compute progress-to-goal reward.
	11.	Apply per-timestep penalty.
	12.	Check for episode termination (collision, goal, timeout).
	13.	Return (observation, reward, done, info).

7.2 Initialization & Sensor Data Pull

def step(self, action):
    # 1. Apply control
    if Config.CONTINUOUS_ACTION:
        steer, throttle, brake = action
        control = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
        self.ego_vehicle.apply_control(control)
    else:
        control = self._map_discrete_action_to_control(action)
        self.ego_vehicle.apply_control(control)

    # 2. Advance simulator
    self.world.tick()

    # 3. Retrieve sensor data
    try:
        sensor_data = self.sensor_interface.get_data()
    except RuntimeError:
        reward = -100
        done = True
        info = {"error": "sensor_timeout"}
        return None, reward, done, info

    # 4. Compute current state
    transform = self.ego_vehicle.get_transform()
    current_loc = transform.location
    velocity = self.ego_vehicle.get_velocity()
    current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    # 5. Initialize reward & info
    reward = 0.0
    info = {
        "collision": False,
        "lane_invasion": False,
        "slow_speed": False,
        "over_speed": False,
        "off_center": False,
        "progress": 0.0,
        "time_penalty": -1,
    }
    done = False
    self.elapsed_steps += 1

7.3 Immediate Collision Penalty

    # 6. Collision event
    if "collision_sensor" in sensor_data:
        reward += Config.COLLISION_REWARD  # –100
        info["collision"] = True
        done = True
        return None, reward, done, info

7.4 Lane Deviation Penalty & Lane-Keeping Bonus

    # 7. Lane invasion event
    if "lane_invasion_sensor" in sensor_data:
        reward += Config.LANE_DEVIATION_REWARD  # –50
        info["lane_invasion"] = True
    else:
        waypoint = self.world.get_map().get_waypoint(current_loc, project_to_road=True)
        center_loc = waypoint.transform.location
        dist_center = current_loc.distance(center_loc)

        if dist_center <= Config.MAX_DISTANCE_FROM_CENTER:
            center_bonus = Config.MAX_DISTANCE_FROM_CENTER - dist_center  # [0, +3]
            reward += center_bonus
        else:
            reward += Config.LANE_DEVIATION_REWARD  # –50
            info["off_center"] = True

7.5 Speed Regulation Reward/Penalty

    # 8. Speed regulation
    if current_speed < Config.MIN_SPEED:
        reward += Config.SLOW_SPEED_REWARD  # –50
        info["slow_speed"] = True
    elif current_speed > Config.MAX_SPEED:
        reward += Config.SPEED_PENALTY  # –1
        info["over_speed"] = True

    speed_error = abs(current_speed - Config.TARGET_SPEED)
    speed_bonus = max(0, 10 - speed_error)  # [0, +10]
    reward += speed_bonus
    info["speed_bonus"] = speed_bonus

7.6 Distance from Center Reward/Penalty

(Already included in section 7.4, combined with lane-keeping logic.)

7.7 Progress Toward Goal

    # 9. Progress-to-goal
    if self.previous_distance_to_goal is None:
        self.previous_distance_to_goal = current_loc.distance(self.goal_location)
        self.initial_distance_to_goal = self.previous_distance_to_goal

    distance_to_goal = current_loc.distance(self.goal_location)
    delta_d = self.previous_distance_to_goal - distance_to_goal

    if delta_d > 0:
        progress_reward = (delta_d / self.initial_distance_to_goal) * 100.0
        reward += progress_reward
        info["progress"] = progress_reward
    else:
        info["progress"] = 0.0

    self.previous_distance_to_goal = distance_to_goal

    # Goal reached
    if distance_to_goal < 2.0:
        reward += 100.0
        info["reached_goal"] = True
        done = True
        return None, reward, done, info

7.8 Time/Efficiency Penalty

    # 10. Time penalty
    reward += Config.SPEED_PENALTY  # –1 per timestep

    if self.elapsed_steps >= Config.MAX_TIMESTEPS:
        reward += -100
        info["timeout"] = True
        done = True
        return None, reward, done, info

7.9 Episode Termination Conditions
	•	Collision: handled in 7.3.
	•	Goal Reached: handled in 7.7.
	•	Timeout: handled in 7.8.

If none of these conditions occur, the episode continues.

    # 11. Return observation, reward, done, info
    obs = self._get_observation()
    return obs, reward, done, info


⸻

8. Sensor Initialization Example in CarlaEnv

# envs/carla_env.py

import random
import math
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import carla

from Connection_manager import ClientConnection
from sensors.Sensor_interface import SensorInterface
from sensors.Sensor_manager import SensorManager
import Config

class CarlaEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # Connect to CARLA server
        conn = ClientConnection(town="Town03")
        self.client, self.world = conn.connect(host=Config.HOST, port=Config.PORT, timeout=Config.TIMEOUT)

        # Create sensor interface
        self.sensor_interface = SensorInterface()

        # Spawn ego vehicle
        blueprint = self.world.get_blueprint_library().find(Config.CAR_NAME)
        spawn_pt = random.choice(self.world.get_map().get_spawn_points())
        self.ego_vehicle = self.world.spawn_actor(blueprint, spawn_pt)

        # Spawn sensors
        collision_attrs = {"type": "sensor.other.collision", "transform": "0,0,0,0,0,0"}
        SensorManager.spawn("collision_sensor", collision_attrs, self.sensor_interface, self.ego_vehicle)

        lane_inv_attrs = {"type": "sensor.other.lane_invasion", "transform": "0,0,0,0,0,0"}
        SensorManager.spawn("lane_invasion_sensor", lane_inv_attrs, self.sensor_interface, self.ego_vehicle)

        imu_attrs = {"type": "sensor.other.imu", "transform": "0,0,0,0,0,0"}
        SensorManager.spawn("imu_sensor", imu_attrs, self.sensor_interface, self.ego_vehicle)

        gnss_attrs = {"type": "sensor.other.gnss", "transform": "0,0,2,0,0,0"}
        SensorManager.spawn("gnss_sensor", gnss_attrs, self.sensor_interface, self.ego_vehicle)

        sem_cam_attrs = {
            "type": Config.SEMANTIC_CAMERA,
            "transform": "1.5,0,2.4,0,0,0",
            "image_size_x": str(Config.IMAGE_WIDTH),
            "image_size_y": str(Config.IMAGE_HEIGHT),
            "fov": str(Config.CAMERA_FOV)
        }
        SensorManager.spawn("sem_seg_camera", sem_cam_attrs, self.sensor_interface, self.ego_vehicle)

        # Define action & observation spaces
        if Config.CONTINUOUS_ACTION:
            self.action_space = spaces.Box(low=np.array([-1.0, 0.0, 0.0]),
                                           high=np.array([1.0, 1.0, 1.0]),
                                           dtype=np.float32)
        else:
            self.action_space = spaces.Discrete(5)

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)

        # Initialize episode variables
        self.elapsed_steps = 0
        start_loc = self.ego_vehicle.get_transform().location
        self.goal_location = self._choose_random_goal(start_loc)
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        self.previous_speed = 0.0
        self.lane_invasion_count = 0

    def reset(self, seed=None, options=None):
        # Destroy existing sensors & vehicle, then reinitialize (omitted for brevity)

        self.elapsed_steps = 0
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        obs = self._get_observation()
        return obs, {}

    def step(self, action):
        # 1. Apply control
        if Config.CONTINUOUS_ACTION:
            steer, throttle, brake = action
            control = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
            self.ego_vehicle.apply_control(control)
        else:
            control = self._map_discrete_action_to_control(action)
            self.ego_vehicle.apply_control(control)

        # 2. Tick simulator
        self.world.tick()

        # 3. Pull sensor data
        try:
            sensor_data = self.sensor_interface.get_data()
        except RuntimeError:
            reward = -100
            done = True
            info = {"error": "sensor_timeout"}
            return None, reward, done, info

        # 4. Get ego state
        transform = self.ego_vehicle.get_transform()
        current_loc = transform.location
        velocity = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        # 5. Initialize reward & info
        reward = 0.0
        info = {
            "collision": False,
            "lane_invasion": False,
            "slow_speed": False,
            "over_speed": False,
            "off_center": False,
            "progress": 0.0,
            "time_penalty": -1,
        }
        done = False
        self.elapsed_steps += 1

        # 6. Collision
        if "collision_sensor" in sensor_data:
            reward += Config.COLLISION_REWARD  # –100
            info["collision"] = True
            done = True
            return None, reward, done, info

        # 7. Lane Deviation & Lane-Keeping
        if "lane_invasion_sensor" in sensor_data:
            reward += Config.LANE_DEVIATION_REWARD  # –50
            info["lane_invasion"] = True
        else:
            waypoint = self.world.get_map().get_waypoint(current_loc, project_to_road=True)
            center_loc = waypoint.transform.location
            dist_center = current_loc.distance(center_loc)

            if dist_center <= Config.MAX_DISTANCE_FROM_CENTER:
                center_bonus = Config.MAX_DISTANCE_FROM_CENTER - dist_center  # [0, +3]
                reward += center_bonus
            else:
                reward += Config.LANE_DEVIATION_REWARD  # –50
                info["off_center"] = True

        # 8. Speed Regulation
        if current_speed < Config.MIN_SPEED:
            reward += Config.SLOW_SPEED_REWARD  # –50
            info["slow_speed"] = True
        elif current_speed > Config.MAX_SPEED:
            reward += Config.SPEED_PENALTY  # –1
            info["over_speed"] = True

        speed_error = abs(current_speed - Config.TARGET_SPEED)
        speed_bonus = max(0, 10 - speed_error)  # [0, +10]
        reward += speed_bonus
        info["speed_bonus"] = speed_bonus

        # 9. Progress Toward Goal
        if self.previous_distance_to_goal is None:
            self.previous_distance_to_goal = current_loc.distance(self.goal_location)
            self.initial_distance_to_goal = self.previous_distance_to_goal

        distance_to_goal = current_loc.distance(self.goal_location)
        delta_d = self.previous_distance_to_goal - distance_to_goal
        if delta_d > 0:
            progress_reward = (delta_d / self.initial_distance_to_goal) * 100.0
            reward += progress_reward
            info["progress"] = progress_reward
        else:
            info["progress"] = 0.0

        self.previous_distance_to_goal = distance_to_goal

        if distance_to_goal < 2.0:
            reward += 100.0
            info["reached_goal"] = True
            done = True
            return None, reward, done, info

        # 10. Time/Efficiency Penalty
        reward += Config.SPEED_PENALTY  # –1 per timestep
        if self.elapsed_steps >= Config.MAX_TIMESTEPS:
            reward += -100
            info["timeout"] = True
            done = True
            return None, reward, done, info

        # 11. Return observation, reward, done, info
        obs = self._get_observation()
        return obs, reward, done, info

    def _get_observation(self):
        transform = self.ego_vehicle.get_transform()
        current_loc = transform.location
        velocity = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        waypoint = self.world.get_map().get_waypoint(current_loc, project_to_road=True)
        center_loc = waypoint.transform.location
        dist_center = current_loc.distance(center_loc)

        dist_to_goal = current_loc.distance(self.goal_location)
        return np.array([current_speed, dist_to_goal, dist_center], dtype=np.float32)

    def _choose_random_goal(self, start_loc):
        all_wps = self.world.get_map().generate_waypoints(distance=5.0)
        far_wps = [wp for wp in all_wps if wp.transform.location.distance(start_loc) > 200.0]
        return random.choice(far_wps).transform.location

    def close(self):
        self.sensor_interface.destroy()
        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()

    def _map_discrete_action_to_control(self, action_idx):
        if action_idx == 0:
            return carla.VehicleControl(throttle=0.5, steer=0.0, brake=0.0)
        elif action_idx == 1:
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.5)
        elif action_idx == 2:
            return carla.VehicleControl(throttle=0.3, steer=-0.5, brake=0.0)
        elif action_idx == 3:
            return carla.VehicleControl(throttle=0.3, steer=0.5, brake=0.0)
        else:
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0)


⸻

9. Tuning & Scaling Guidelines
	1.	Reward Magnitudes
	•	Collision (COLLISION_REWARD) is set to –100 to outweigh minor positive rewards.
	•	Lane Deviation (LANE_DEVIATION_REWARD) is –50 to strongly discourage lane breaches.
	•	Speed Bonuses/Penalties:
	•	SLOW_SPEED_REWARD = –50 for driving below MIN_SPEED.
	•	SPEED_PENALTY = –1 per timestep for driving above MAX_SPEED.
	•	speed_bonus up to +10 for matching TARGET_SPEED.
	2.	Progress Scaling
	•	progress_reward uses (delta_d / initial_distance_to_goal) * 100.0. Adjust multiplier if distances are large or small.
	3.	Lane-Keeping Bonus Range
	•	center_bonus = MAX_DISTANCE_FROM_CENTER – dist_center yields up to +3 per timestep. Multiply by a larger factor (e.g., ×5) if needed for stable learning.
	4.	Time Penalty
	•	A flat –1 per timestep encourages faster completion. Increase to –2 if the agent loiters.
	5.	Episode Length
	•	MAX_TIMESTEPS = 7500 corresponds to ~375 seconds at 20 Hz. Adjust for desired episode duration.
	6.	Goal-Reaching Bonus
	•	Set to +100. Increase if reaching goal is rare, decrease if too easy.
	7.	Curriculum Learning
	1.	Begin with no traffic and straight routes.
	2.	Add static obstacles.
	3.	Add NPC vehicles via Traffic_manager.py.
	4.	Add pedestrians.
	•	Adjust reward weights as complexity increases.

⸻

10. Appendix: Helper Methods & CARLA API Snippets

10.1 Distance to Lane Center

def lateral_distance_to_lane_center(world, location):
    waypoint = world.get_map().get_waypoint(location, project_to_road=True)
    center_loc = waypoint.transform.location
    return location.distance(center_loc)

10.2 Angle Between Vectors (Optional)

def angle_between_vectors(v1, v2):
    dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
    mag1 = math.sqrt(v1.x**2 + v1.y**2 + v1.z**2)
    mag2 = math.sqrt(v2.x**2 + v2.y**2 + v2.z**2)
    cos_angle = dot / (mag1 * mag2 + 1e-8)
    return math.degrees(math.acos(np.clip(cos_angle, -1.0, 1.0)))

10.3 Generating a Waypoint Route (Optional)

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

def compute_global_route(world_map, start_wp, goal_wp):
    dao = GlobalRoutePlannerDAO(world_map, sampling_resolution=2.0)
    grp = GlobalRoutePlanner(dao)
    grp.setup()
    route = grp.trace_route(start_wp.transform.location, goal_wp.transform.location)
    return [wp for wp, _ in route]


⸻

11. References
	1.	CARLA Simulator Documentation (Python API):
https://carla.readthedocs.io/en/latest/python_api/
	2.	Gymnasium API Reference:
https://gymnasium.farama.org/
	3.	CARLA Agent Navigation & Global Route Planner:
	•	agents/navigation/global_route_planner.py
	•	agents/navigation/global_route_planner_dao.py
	4.	CARLA Sensor Tutorials:
	•	https://carla.readthedocs.io/en/latest/tuto_Gym_env/
	•	https://carla.readthedocs.io/en/latest/tuto_sensors/

