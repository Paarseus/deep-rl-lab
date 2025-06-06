import numpy as np
import carla
from typing import Dict, Tuple, Optional, Any, List
from dataclasses import dataclass
from collections import deque
import math


@dataclass
class RewardWeights:
    """Configuration for reward component weights - based on reward engineering table"""
    
    # === POSITIVE REWARDS ===
    # Goal-oriented
    goal_reached: float = 100.0
    progress_toward_goal: float = 1.0  # scaled by distance
    
    # Lane keeping
    lane_keeping_centered: float = 3.0  # max 3.0 when perfectly centered
    correct_lane_change: float = 5.0
    
    # Speed management
    target_speed_proximity: float = 10.0  # max when at target speed
    smooth_acceleration: float = 4.0
    
    # Steering
    steady_steering: float = 1.0
    
    # Traffic rules
    proper_stop_at_red: float = 5.0
    proper_stop_at_stop_sign: float = 5.0
    yielding_to_pedestrian: float = 5.0
    
    # Safe driving
    safe_following_distance: float = 2.0
    
    # === NEGATIVE REWARDS (PENALTIES) ===
    # Collisions
    collision_any: float = -100.0
    collision_pedestrian: float = -150.0
    collision_vehicle: float = -100.0
    collision_static: float = -100.0
    
    # Lane violations
    lane_invasion: float = -50.0
    off_road: float = -50.0
    wrong_direction: float = -50.0
    
    # Speed violations
    below_minimum_speed: float = -50.0
    above_maximum_speed: float = -1.0  # per timestep
    hard_braking: float = -10.0
    hard_acceleration: float = -10.0
    stalling: float = -10.0
    
    # Traffic violations
    running_red_light: float = -20.0
    running_stop_sign: float = -20.0
    
    # Following distance
    tailgating: float = -10.0  # per second
    
    # Configuration parameters
    target_speed: float = 22.0  # m/s (~80 km/h)
    min_speed: float = 15.0  # m/s (~54 km/h)
    max_speed: float = 25.0  # m/s (~90 km/h)
    max_distance_from_center: float = 3.0  # meters
    safe_following_distance: float = 5.0  # meters
    min_following_distance: float = 2.0  # meters for tailgating


class RewardManager:
    """
    Comprehensive reward function for CARLA RL environment.
    Based on the reward engineering table with focus on essential driving behaviors.
    """
    
    def __init__(self, hero_manager, world, goal_location: Optional[carla.Location] = None, 
                 weights: Optional[RewardWeights] = None):
        """
        Initialize reward function.
        
        Args:
            hero_manager: HeroManager instance for accessing vehicle
            world: CARLA world instance
            goal_location: Target destination (if None, distance rewards disabled)
            weights: Optional custom reward weights
        """
        self.hero_manager = hero_manager
        self.world = world
        self.weights = weights or RewardWeights()
        self.goal_location = goal_location
        
        # State tracking
        self.previous_location = None
        self.previous_speed = 0.0
        self.previous_steer = 0.0
        self.previous_throttle = 0.0
        self.previous_brake = 0.0
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        
        # Event tracking
        self.collision_intensity = 0.0
        self.collision_actor = None
        self.lane_invasion_event = False
        self.at_red_light = False
        self.at_stop_sign = False
        self.stopped_at_red_duration = 0.0
        self.stopped_at_stop_duration = 0.0
        
        # History tracking
        self.speed_history = deque(maxlen=50)  # ~5 seconds at 10Hz
        self.steer_history = deque(maxlen=10)
        self.acceleration_history = deque(maxlen=10)
        
        # Lane change tracking
        self.previous_lane_id = None
        self.lane_change_completed = False
        
    def reset(self):
        """Reset reward function state for new episode"""
        self.previous_location = None
        self.previous_speed = 0.0
        self.previous_steer = 0.0
        self.previous_throttle = 0.0
        self.previous_brake = 0.0
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        
        self.collision_intensity = 0.0
        self.collision_actor = None
        self.lane_invasion_event = False
        self.at_red_light = False
        self.at_stop_sign = False
        self.stopped_at_red_duration = 0.0
        self.stopped_at_stop_duration = 0.0
        
        self.speed_history.clear()
        self.steer_history.clear()
        self.acceleration_history.clear()
        
        self.previous_lane_id = None
        self.lane_change_completed = False
        
    def set_goal_location(self, goal_location: carla.Location):
        """Update goal location for distance-based rewards"""
        self.goal_location = goal_location
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        
    def compute_reward(self, sensor_data: Dict[str, Tuple[int, Any]], 
                      control: carla.VehicleControl) -> Tuple[float, Dict[str, float]]:
        """
        Compute total reward and individual components based on reward engineering table.
        
        Args:
            sensor_data: Dictionary of sensor data from hero_manager.get_sensor_data()
            control: Vehicle control applied this step
            
        Returns:
            total_reward: Total reward value
            reward_components: Dictionary of individual reward components
        """
        hero = self.hero_manager.hero
        if hero is None:
            return 0.0, {}
            
        # Get current vehicle state
        transform = hero.get_transform()
        velocity = hero.get_velocity()
        location = transform.location
        
        # Calculate speed
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        speed_kmh = speed * 3.6
        
        # Update histories
        self.speed_history.append(speed)
        self.steer_history.append(control.steer)
        
        # Initialize location tracking
        if self.previous_location is None:
            self.previous_location = location
            
        # Update sensor states
        self._update_collision_state(sensor_data)
        self._update_lane_invasion_state(sensor_data)
        
        # Initialize reward components
        rewards = {}
        
        # === POSITIVE REWARDS ===
        
        # 1. Goal-oriented rewards
        if self.goal_location is not None:
            goal_reward, reached = self._compute_goal_rewards(location)
            rewards['goal_progress'] = goal_reward
            if reached:
                rewards['goal_reached'] = self.weights.goal_reached
        
        # 2. Lane keeping rewards
        lane_reward, lane_id = self._compute_lane_keeping_reward(transform)
        rewards['lane_keeping'] = lane_reward
        
        # 3. Lane change detection
        if self._detect_correct_lane_change(lane_id):
            rewards['correct_lane_change'] = self.weights.correct_lane_change
            
        # 4. Speed rewards
        speed_reward = self._compute_speed_proximity_reward(speed)
        rewards['speed_proximity'] = speed_reward
        
        # 5. Smooth driving rewards
        smooth_accel_reward = self._compute_smooth_acceleration_reward(speed, control)
        rewards['smooth_acceleration'] = smooth_accel_reward
        
        steady_steer_reward = self._compute_steady_steering_reward(control)
        rewards['steady_steering'] = steady_steer_reward
        
        # 6. Safe following distance
        following_reward = self._compute_following_distance_reward(transform)
        rewards['safe_following'] = following_reward
        
        # 7. Traffic light/stop sign rewards
        traffic_rewards = self._compute_traffic_compliance_rewards(transform, speed)
        rewards.update(traffic_rewards)
        
        # === NEGATIVE REWARDS (PENALTIES) ===
        
        # 8. Collision penalties
        collision_penalty = self._compute_collision_penalty()
        if collision_penalty < 0:
            rewards['collision'] = collision_penalty
            
        # 9. Lane invasion penalty
        if self.lane_invasion_event:
            rewards['lane_invasion'] = self.weights.lane_invasion
            self.lane_invasion_event = False
            
        # 10. Off-road penalty
        off_road_penalty = self._compute_off_road_penalty(location)
        if off_road_penalty < 0:
            rewards['off_road'] = off_road_penalty
            
        # 11. Wrong direction penalty
        wrong_dir_penalty = self._compute_wrong_direction_penalty(transform)
        if wrong_dir_penalty < 0:
            rewards['wrong_direction'] = wrong_dir_penalty
            
        # 12. Speed violation penalties
        speed_penalties = self._compute_speed_penalties(speed, control)
        rewards.update(speed_penalties)
        
        # 13. Traffic violation penalties
        traffic_penalties = self._compute_traffic_violation_penalties(transform, speed)
        rewards.update(traffic_penalties)
        
        # 14. Tailgating penalty
        tailgating_penalty = self._compute_tailgating_penalty(transform)
        if tailgating_penalty < 0:
            rewards['tailgating'] = tailgating_penalty
            
        # Update previous states
        self.previous_location = location
        self.previous_speed = speed
        self.previous_steer = control.steer
        self.previous_throttle = control.throttle
        self.previous_brake = control.brake
        self.previous_lane_id = lane_id
        
        # Calculate total reward
        total_reward = sum(rewards.values())
        
        return total_reward, rewards
    
    # === POSITIVE REWARD METHODS ===
    
    def _compute_goal_rewards(self, location: carla.Location) -> Tuple[float, bool]:
        """Compute goal-related rewards"""
        if self.goal_location is None:
            return 0.0, False
            
        distance = location.distance(self.goal_location)
        
        # Initialize distances
        if self.initial_distance_to_goal is None:
            self.initial_distance_to_goal = distance
            self.previous_distance_to_goal = distance
            
        # Check if goal reached
        if distance < 2.0:  # Within 2 meters
            return 0.0, True
            
        # Progress reward
        progress = self.previous_distance_to_goal - distance
        if self.initial_distance_to_goal > 0:
            normalized_progress = (progress / self.initial_distance_to_goal) * 100
            progress_reward = self.weights.progress_toward_goal * normalized_progress
        else:
            progress_reward = 0.0
            
        self.previous_distance_to_goal = distance
        return progress_reward, False
    
    def _compute_lane_keeping_reward(self, transform: carla.Transform) -> Tuple[float, int]:
        """Compute lane keeping centered reward"""
        waypoint = self.world.get_map().get_waypoint(
            transform.location, 
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        if waypoint is None:
            return 0.0, -1
            
        # Calculate distance from lane center
        waypoint_loc = waypoint.transform.location
        dist_center = transform.location.distance(waypoint_loc)
        
        # Reward decreases linearly with distance from center
        if dist_center <= self.weights.max_distance_from_center:
            reward = self.weights.lane_keeping_centered * (1.0 - dist_center / self.weights.max_distance_from_center)
        else:
            reward = 0.0
            
        return reward, waypoint.lane_id
    
    def _detect_correct_lane_change(self, current_lane_id: int) -> bool:
        """Detect if a correct lane change was completed"""
        if self.previous_lane_id is None or current_lane_id == -1:
            return False
            
        # Lane change detected
        if self.previous_lane_id != current_lane_id and not self.lane_invasion_event:
            # This is a correct lane change (no lane invasion triggered)
            return True
            
        return False
    
    def _compute_speed_proximity_reward(self, speed: float) -> float:
        """Compute reward for maintaining target speed"""
        speed_diff = abs(speed - self.weights.target_speed)
        
        # Maximum reward when exactly at target speed, decreasing linearly
        if speed_diff <= 10.0:  # Within 10 m/s of target
            reward = self.weights.target_speed_proximity * (1.0 - speed_diff / 10.0)
        else:
            reward = 0.0
            
        return reward
    
    def _compute_smooth_acceleration_reward(self, speed: float, control: carla.VehicleControl) -> float:
        """Compute reward for smooth acceleration (low jerk)"""
        if len(self.speed_history) < 2:
            return 0.0
            
        # Calculate acceleration
        dt = 0.1  # Assuming 10Hz
        current_accel = (speed - self.speed_history[-2]) / dt
        
        if len(self.acceleration_history) > 0:
            prev_accel = self.acceleration_history[-1]
            jerk = abs(current_accel - prev_accel) / dt
            
            # Reward for low jerk
            if jerk < 2.0:  # m/s³
                reward = self.weights.smooth_acceleration * (1.0 - jerk / 2.0)
            else:
                reward = 0.0
        else:
            reward = 0.0
            
        self.acceleration_history.append(current_accel)
        return reward
    
    def _compute_steady_steering_reward(self, control: carla.VehicleControl) -> float:
        """Compute reward for steady steering"""
        if len(self.steer_history) < 2:
            return 0.0
            
        # Calculate steering rate (degrees per second)
        dt = 0.1  # Assuming 10Hz
        steer_rate = abs(control.steer - self.steer_history[-2]) / dt * 180.0 / np.pi
        
        # Reward for low steering rate
        if steer_rate < 15.0:  # degrees/second
            reward = self.weights.steady_steering * (1.0 - steer_rate / 15.0)
        else:
            reward = 0.0
            
        return reward
    
    def _compute_following_distance_reward(self, transform: carla.Transform) -> float:
        """Compute reward for maintaining safe following distance"""
        # Simple proximity check using location
        # In a full implementation, you'd use radar or LIDAR data
        hero = self.hero_manager.hero
        
        # Get vehicles nearby
        vehicles = self.world.get_actors().filter('vehicle.*')
        min_distance = float('inf')
        
        for vehicle in vehicles:
            if vehicle.id == hero.id:
                continue
                
            # Check if vehicle is in front
            vehicle_loc = vehicle.get_location()
            forward_vec = transform.get_forward_vector()
            
            # Vector to other vehicle
            dx = vehicle_loc.x - transform.location.x
            dy = vehicle_loc.y - transform.location.y
            
            # Dot product to check if in front
            dot = dx * forward_vec.x + dy * forward_vec.y
            
            if dot > 0:  # Vehicle is in front
                distance = transform.location.distance(vehicle_loc)
                min_distance = min(min_distance, distance)
        
        # Reward for safe distance
        if min_distance > self.weights.safe_following_distance:
            return self.weights.safe_following_distance
        elif min_distance > self.weights.min_following_distance:
            # Partial reward
            return self.weights.safe_following_distance * (min_distance - self.weights.min_following_distance) / \
                   (self.weights.safe_following_distance - self.weights.min_following_distance)
        else:
            return 0.0
    
    def _compute_traffic_compliance_rewards(self, transform: carla.Transform, speed: float) -> Dict[str, float]:
        """Compute rewards for proper stops at traffic lights and stop signs"""
        rewards = {}
        hero = self.hero_manager.hero
        
        # Check traffic lights
        if hero.is_at_traffic_light():
            traffic_light = hero.get_traffic_light()
            if traffic_light and traffic_light.get_state() == carla.TrafficLightState.Red:
                self.at_red_light = True
                if speed < 0.5:  # Stopped
                    self.stopped_at_red_duration += 0.1
                    if self.stopped_at_red_duration >= 1.0:  # Stopped for 1 second
                        rewards['proper_stop_red'] = self.weights.proper_stop_at_red
                else:
                    self.stopped_at_red_duration = 0.0
            else:
                self.at_red_light = False
                self.stopped_at_red_duration = 0.0
        
        # Check stop signs (simplified - would need proper implementation)
        # This would require checking waypoints for stop sign information
        
        return rewards
    
    # === NEGATIVE REWARD METHODS ===
    
    def _compute_collision_penalty(self) -> float:
        """Compute collision penalties based on collision type"""
        if self.collision_intensity > 0 and self.collision_actor:
            penalty = 0.0
            
            # Determine collision type
            if 'walker' in self.collision_actor.type_id:
                penalty = self.weights.collision_pedestrian
            elif 'vehicle' in self.collision_actor.type_id:
                penalty = self.weights.collision_vehicle
            else:
                penalty = self.weights.collision_static
                
            # Reset collision state
            self.collision_intensity = 0.0
            self.collision_actor = None
            
            return penalty
            
        return 0.0
    
    def _compute_off_road_penalty(self, location: carla.Location) -> float:
        """Compute penalty for being off-road"""
        waypoint = self.world.get_map().get_waypoint(
            location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        if waypoint is None:
            return self.weights.off_road
            
        # Check distance from road
        dist_center = location.distance(waypoint.transform.location)
        
        if dist_center > self.weights.max_distance_from_center:
            return self.weights.off_road
            
        return 0.0
    
    def _compute_wrong_direction_penalty(self, transform: carla.Transform) -> float:
        """Compute penalty for driving in wrong direction"""
        waypoint = self.world.get_map().get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        if waypoint is None:
            return 0.0
            
        # Compare vehicle heading with lane direction

        lane_forward = waypoint.transform.get_forward_vector()
        vehicle_forward = transform.get_forward_vector()
        
        # Dot product
        dot = lane_forward.x * vehicle_forward.x + lane_forward.y * vehicle_forward.y
        
        # If dot product is negative, we're going wrong way
        if dot < -0.5:  # more than 120 degrees 
            return self.weights.wrong_direction
            
        return 0.0
    
    def _compute_speed_penalties(self, speed: float, control: carla.VehicleControl) -> Dict[str, float]:
        """speed penalties"""
        penalties = {}
        
        # Below minimum speed
        if speed < self.weights.min_speed and len(self.speed_history) > 30:
            # Check if we've been slow for a while (not just starting)
            avg_speed = np.mean(list(self.speed_history)[-30:])
            if avg_speed < self.weights.min_speed:
                penalties['below_min_speed'] = self.weights.below_minimum_speed
        
        # Above maximum speed
        if speed > self.weights.max_speed:
            penalties['above_max_speed'] = self.weights.above_maximum_speed
        
        # Hard braking
        if self.previous_brake < 0.3 and control.brake > 0.7:
            penalties['hard_braking'] = self.weights.hard_braking
        
        # Hard acceleration
        if self.previous_throttle < 0.3 and control.throttle > 0.7:
            penalties['hard_acceleration'] = self.weights.hard_acceleration
        
        # Stalling (very low speed for extended time)
        if len(self.speed_history) == 50:  # Full history
            recent_speeds = list(self.speed_history)[-50:]
            if all(s < 0.5 for s in recent_speeds[-50:]):  # 5 seconds of no movement
                if not self.at_red_light and not self.at_stop_sign:
                    penalties['stalling'] = self.weights.stalling
        
        return penalties
    
    def _compute_traffic_violation_penalties(self, transform: carla.Transform, speed: float) -> Dict[str, float]:
        """Compute penalties for traffic violations"""
        penalties = {}
        hero = self.hero_manager.hero
        
        # Running red light
        if hero.is_at_traffic_light():
            traffic_light = hero.get_traffic_light()
            if traffic_light and traffic_light.get_state() == carla.TrafficLightState.Red:
                if speed > 1.0:  # Moving through red light
                    penalties['running_red'] = self.weights.running_red_light
        
        # Running stop sign would require additional implementation
        
        return penalties
    
    def _compute_tailgating_penalty(self, transform: carla.Transform) -> float:
        """Compute penalty for tailgating"""
        hero = self.hero_manager.hero
        vehicles = self.world.get_actors().filter('vehicle.*')
        
        for vehicle in vehicles:
            if vehicle.id == hero.id:
                continue
                
            # Check if vehicle is in front
            vehicle_loc = vehicle.get_location()
            forward_vec = transform.get_forward_vector()
            
            # Vector to other vehicle
            dx = vehicle_loc.x - transform.location.x
            dy = vehicle_loc.y - transform.location.y
            
            # Dot product to check if in front
            dot = dx * forward_vec.x + dy * forward_vec.y
            
            if dot > 0:  # Vehicle is in front
                distance = transform.location.distance(vehicle_loc)
                if distance < self.weights.min_following_distance:
                    # Check our speed
                    velocity = hero.get_velocity()
                    speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                    
                    if speed > 5.0:  # Only penalize if moving
                        return self.weights.tailgating
        
        return 0.0
    
    # === HELPER METHODS ===
    
    def _update_collision_state(self, sensor_data: Dict[str, Tuple[int, Any]]):
        """Update collision state from sensor data"""
        if 'collision' in sensor_data:
            _, collision_data = sensor_data['collision']
            if collision_data and len(collision_data) > 1:
                self.collision_actor = collision_data[0]
                self.collision_intensity = collision_data[1]
    
    def _update_lane_invasion_state(self, sensor_data: Dict[str, Tuple[int, Any]]):
        """Update lane invasion state from sensor data"""
        if 'lane_invasion' in sensor_data:
            _, lane_data = sensor_data['lane_invasion']
            if lane_data and len(lane_data) > 1:
                crossed_markings = lane_data[1]
                if crossed_markings:
                    self.lane_invasion_event = True
    
    def get_info(self) -> Dict[str, Any]:
        """Get additional information about the current state"""
        info = {
            'current_speed': self.previous_speed,
            'distance_to_goal': self.previous_distance_to_goal if self.previous_distance_to_goal else -1,
            'at_red_light': self.at_red_light,
            'at_stop_sign': self.at_stop_sign,
        }
        
        if len(self.speed_history) > 0:
            info['avg_speed'] = np.mean(self.speed_history)
            
        return info
