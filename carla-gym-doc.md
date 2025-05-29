## Carla Simulator Gymnasium Environment
Documention: https://gymnasium.farama.org/introduction/create_custom_env/ <br>

```
Environment Creation:
__init__() → Set up everything

Training Loop (repeats thousands of times):
reset() → Start episode → return initial_observation
   ↓
step() → Process action → _update_navigation() → _calculate_reward_and_termination() → return obs, reward, done
   ↓
step() → step() → step() → ... (until episode ends)
   ↓
reset() → Start new episode

Training End:
close() → Clean up everything
```

### 1. ```__init__()```
```python 
def __init__(self, client, world, town, checkpoint_frequency=100, continuous_action=True) -> None:
```
### Description:
Sets up the entire environment infrastructure. Defines what observations and actions look like for the RL agent, establishes CARLA connections, initializes all tracking variables, and creates pedestrians in the world. <br>

### Goals:
Initialize the CARLA gymnasium environment with all necessary spaces, variables, and connections. <br>

### Input Variables:
```client``` - CARLA client connection object <br>
```world``` - CARLA world object <br>
```town``` - Map name (e.g., "Town10HD_Opt") <br>
```checkpoint_frequency``` - How often to save progress (default: 100) <br>
```continuous_action``` - Use continuous vs discrete actions (default: True) <br>

### Variable Sets:
```python
# Connection variables
self.client, self.world, self.blueprint_library, self.map, self.town

# Configuration variables (Available in settings.py)
self.continuous_action_space, self.display_on, self.checkpoint_frequency, self.fresh_start

# Gymnasium spaces (CRITICAL for RL)
self.observation_space = Dict({'image': Box(...), 'navigation': Box(...)})
self.action_space = Box(...) or Discrete(...)

# Actor management lists (For keeping track of spawned actors, senesors and walkers)
self.sensor_list, self.actor_list, self.walker_list = [], [], []

# Sensor objects (From sensors.py)
self.camera_obj, self.env_camera_obj, self.collision_obj = None, None, None

# Navigation variables
self.route_waypoints, self.current_waypoint_index, self.checkpoint_waypoint_index = None, 0, 0

# Vehicle state variables
self.velocity, self.throttle, self.previous_steer = 0.0, 0.0, 0.0
self.rotation, self.location, self.previous_location = 0.0, None, None

# Performance metrics
self.distance_from_center, self.center_lane_deviation, self.angle = 0.0, 0.0, 0.0

# Episode management  
self.timesteps, self.episode_start_time, self.collision_history = 0, 0, None

# Driving parameters (tunable)
self.target_speed, self.max_speed, self.min_speed = 22.0, 25.0, 15.0
self.max_distance_from_center = 3.0

# Observations
self.image_obs, self.navigation_obs = None, None
```
### Returns:
Nothing. <br>

### How It's Used:
Called once when creating the environment. Sets up everything needed for training. <br>

### 2. ```reset()``` 
```python
reset(self, seed=None, options=None)
```
### Description:
1. Cleans up any existing actors from previous episode
2. Spawns new vehicle at fixed spawn point
3. Attaches camera and collision sensors to vehicle
4. Generates route waypoints from spawn to destination
5. Resets all episode counters and metrics
6. Creates initial observation dictionary
7. Returns observation and info for RL agent

### Goals:
Reset the environment to start a new episode. Spawn vehicle, attach sensors, generate route, return initial observation. <br>

### Input Variables:
```seed``` - Random seed for reproducibility (optional) <br>
```options``` - Additional reset options (optional) <br>

### Variable Sets:
```python
# Cleanup variables
self.sensor_list, self.actor_list  # For destroying previous actors

# Spawning variables  
vehicle_bp = self.get_vehicle(CAR_NAME)  # Vehicle blueprint
transform = self.map.get_spawn_points()[19]  # Spawn location
self.total_distance = 750  # Route length

# Sensor creation
self.camera_obj = CameraSensor(self.vehicle)
self.env_camera_obj = CameraSensorEnv(self.vehicle)  
self.collision_obj = CollisionSensor(self.vehicle)

# Episode initialization
self.timesteps = 0
self.episode_start_time = time.time()
self.rotation = self.vehicle.get_transform().rotation.yaw
self.previous_location = self.vehicle.get_location()

# Reset counters
self.center_lane_deviation = 0.0
self.throttle, self.previous_steer, self.velocity = 0.0, 0.0, 0.0
self.distance_from_center, self.angle, self.distance_covered = 0.0, 0.0, 0.0

# Observation creation
self.navigation_obs = np.array([throttle, velocity, steer, distance, angle])
observation = {'image': self.image_obs, 'navigation': self.navigation_obs}
info = {'distance_covered': 0, 'current_waypoint_index': self.current_waypoint_index}
```

### Returns:
```python
return observation, info
# observation = {'image': (160,80,3) array, 'navigation': (5,) array}
# info = {'distance_covered': int, 'center_lane_deviation': float, ...}
```
### How its used:
Called by RL training loop at start of each episode. Agent receives initial observation to make first action decision.<br>

### 3. ```step()```
```python
def step(self, action):
```
### Description:
1. Converts RL action to vehicle control commands
2. Applies smoothed control to vehicle for stability
3. Updates vehicle state (position, velocity, rotation)
4. Updates navigation state (waypoint progress, lane position)
5. Calculates reward based on driving performance
6. Checks termination conditions (collision, completion, timeout)
7. Creates new observation from sensors and state
8. Returns observation, reward, and episode status

### Goals:
Execute one timestep of the environment. Apply agent's action, simulate physics, calculate reward, return new observation.<br>

### Input Variables:
```action``` - Agent's chosen action (steering/throttle for continuous, or index for discrete) <br>

### Variable Sets:
```python
# Action processing
if continuous_action_space:
    steer = np.clip(float(action[0]), -1.0, 1.0)  # Steering input
    throttle = np.clip(float((action[1] + 1.0) / 2), 0.0, 1.0)  # Throttle input
else:
    steer = self.get_discrete_action_space()[action]  # Discrete steering

# Vehicle control
final_steer = self.previous_steer * 0.9 + steer * 0.1  # Smoothed steering
final_throttle = self.throttle * 0.9 + throttle * 0.1  # Smoothed throttle
self.vehicle.apply_control(carla.VehicleControl(steer=final_steer, throttle=final_throttle))

# State updates
velocity_vector = self.vehicle.get_velocity()
self.velocity = np.sqrt(velocity_vector.x**2 + velocity_vector.y**2 + velocity_vector.z**2) * 3.6

# Navigation updates (from _update_navigation)
self.rotation = self.vehicle.get_transform().rotation.yaw
self.location = self.vehicle.get_location()
self.current_waypoint_index  # Updated based on progress
self.distance_from_center = distance_to_line(...)  # Lane deviation
self.angle = angle_diff(vehicle_forward, road_forward)  # Alignment

# Reward calculation (from _calculate_reward_and_termination)
if collision: reward, terminated = -100, True
elif off_road: reward, terminated = -50, True  
elif route_complete: reward, terminated = 100, True
else: reward = speed_factor * centering_factor * angle_factor

# Observation creation
normalized_velocity = self.velocity / self.target_speed
normalized_distance = self.distance_from_center / self.max_distance_from_center
normalized_angle = self.angle / np.deg2rad(20)
self.navigation_obs = np.array([throttle, normalized_velocity, steer, normalized_distance, normalized_angle])
observation = {'image': self.image_obs, 'navigation': self.navigation_obs}
```

### Returns:
```python
return observation, reward, terminated, truncated, info
# observation = {'image': (160,80,3) array, 'navigation': (5,) array}
# reward = float (positive for good driving, negative for bad)
# terminated = bool (episode ended due to success/failure)
# truncated = bool (episode ended due to time limit)
# info = dict with detailed metrics
```

### How It's Used:
Called by RL training loop every timestep. Agent uses observation to decide next action. Reward is used to update neural network. Episode ends if terminated/truncated. <br>

### 4. ```close()```
```python
def close(self):
```

### Description:
Destroys all CARLA actors and closes pygame display to free resources.<br>

### Goals:
Clean up all environment resources when training is finished. <br>

### Variables:
```python
self.sensor_list, self.actor_list  # For cleanup
self.display_on  # Whether to quit pygame
```

### Returns:
Nothing. <br>

### How It's Used:
Called at the start of each reset() to clean up before spawning new actors. <br>

