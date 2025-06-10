import numpy as np
import carla
from typing import Dict, Tuple, Optional, Any, List
from dataclasses import dataclass
from collections import deque
import math


@dataclass
class RewardWeights:
    """Essential reward weights - only the absolutely necessary ones"""
    
    # === POSITIVE REWARDS ===
    lane_keeping_centered: float = 3.0
    progress_toward_goal: float = 1.0
    target_speed_proximity: float = 5.0
    
    # === NEGATIVE REWARDS (PENALTIES) ===
    collision_any: float = -100.0
    lane_invasion: float = -20.0
    below_minimum_speed: float = -10.0
    above_maximum_speed: float = -10.0
    
    # === CONFIGURATION ===
    target_speed: float = 22.0
    min_speed: float = 10.0
    max_speed: float = 35.0
    max_distance_from_center: float = 2.5


class RewardManager:
    """
    Simplified reward function with only essential driving behaviors:
    1. Avoid collisions
    2. Stay in lane  
    3. Make forward progress
    4. Maintain reasonable speed
    """
    
    def __init__(self, hero_manager, world, goal_location: Optional[carla.Location] = None, 
                 weights: Optional[RewardWeights] = None):
        self.hero_manager = hero_manager
        self.world = world
        self.weights = weights or RewardWeights()
        self.goal_location = goal_location
        
        # State tracking
        self.previous_location = None
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        
        # Event tracking
        self.collision_intensity = 0.0
        self.collision_actor = None
        self.lane_invasion_event = False
        
    def reset(self):
        """Reset reward function state for new episode"""
        self.previous_location = None
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        self.collision_intensity = 0.0
        self.collision_actor = None
        self.lane_invasion_event = False
        
    def set_goal_location(self, goal_location: carla.Location):
        """Update goal location for distance-based rewards"""
        self.goal_location = goal_location
        self.previous_distance_to_goal = None
        self.initial_distance_to_goal = None
        
    def compute_reward(self, sensor_data: Dict[str, Tuple[int, Any]], 
                      control: carla.VehicleControl) -> Tuple[float, Dict[str, float]]:
        """Compute total reward with only essential components"""
        hero = self.hero_manager.hero
        if hero is None:
            return 0.0, {}
            
        # Get current vehicle state
        transform = hero.get_transform()
        velocity = hero.get_velocity()
        location = transform.location
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Initialize location tracking
        if self.previous_location is None:
            self.previous_location = location
            
        # Update sensor states
        self._update_collision_state(sensor_data)
        self._update_lane_invasion_state(sensor_data)
        
        rewards = {}
        
        # === POSITIVE REWARDS ===
        
        # Goal-oriented rewards
        if self.goal_location is not None:
            goal_reward = self._compute_goal_rewards(location)
            if goal_reward > 0:
                rewards['goal_progress'] = goal_reward  # Reward for moving toward destination
        
        # Lane keeping rewards
        lane_reward = self._compute_lane_keeping_reward(transform)
        if lane_reward > 0:
            rewards['lane_keeping'] = lane_reward  # Reward for staying centered in lane
            
        # Speed rewards
        speed_reward = self._compute_speed_reward(speed)
        if speed_reward > 0:
            rewards['target_speed'] = speed_reward  # Reward for maintaining optimal speed
        
        # === NEGATIVE REWARDS (PENALTIES) ===
        
        # Collision penalties
        collision_penalty = self._compute_collision_penalty()
        if collision_penalty < 0:
            rewards['collision'] = collision_penalty  # Penalty for any collision
            
        # Lane violation penalties
        if self.lane_invasion_event:
            rewards['lane_invasion'] = self.weights.lane_invasion  # Penalty for crossing lane markings
            self.lane_invasion_event = False
            
        # Speed violation penalties
        speed_penalty = self._compute_speed_reward(speed)
        if speed_penalty < 0:
            rewards['speed_violation'] = speed_penalty  # Penalty for driving too fast/slow
        
        # Update previous states
        self.previous_location = location
        
        # Calculate total reward
        total_reward = sum(rewards.values())
        
        return total_reward, rewards
    
    def _compute_goal_rewards(self, location: carla.Location) -> float:
        """Compute progress toward goal"""
        if self.goal_location is None:
            return 0.0
            
        distance = location.distance(self.goal_location)
        
        # Initialize distances
        if self.initial_distance_to_goal is None:
            self.initial_distance_to_goal = distance
            self.previous_distance_to_goal = distance
            return 0.0
            
        # Progress reward
        progress = self.previous_distance_to_goal - distance
        if self.initial_distance_to_goal > 0:
            normalized_progress = (progress / self.initial_distance_to_goal) * 100
            progress_reward = self.weights.progress_toward_goal * normalized_progress
        else:
            progress_reward = 0.0
            
        self.previous_distance_to_goal = distance
        return progress_reward
    
    def _compute_lane_keeping_reward(self, transform: carla.Transform) -> float:
        """Compute lane keeping reward"""
        try:
            waypoint = self.world.get_map().get_waypoint(
                transform.location, 
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            
            if waypoint is None:
                return 0.0
                
            # Calculate distance from lane center
            waypoint_loc = waypoint.transform.location
            dist_center = transform.location.distance(waypoint_loc)
            
            # Reward decreases linearly with distance from center
            if dist_center <= self.weights.max_distance_from_center:
                reward = self.weights.lane_keeping_centered * (1.0 - dist_center / self.weights.max_distance_from_center)
                return reward
            else:
                return 0.0
        except:
            return 0.0
    
    def _compute_speed_reward(self, speed: float) -> float:
        """Compute speed-related rewards and penalties"""
        # Penalty for being too slow
        if speed < self.weights.min_speed:
            return self.weights.below_minimum_speed
            
        # Penalty for being too fast
        elif speed > self.weights.max_speed:
            return self.weights.above_maximum_speed
            
        # Reward for being near target speed
        else:
            speed_diff = abs(speed - self.weights.target_speed)
            if speed_diff < 10.0:  # Within 10 m/s of target
                reward = self.weights.target_speed_proximity * (1.0 - speed_diff / 10.0)
                return reward
                
        return 0.0
    
    def _compute_collision_penalty(self) -> float:
        """Compute collision penalties"""
        if self.collision_intensity > 0:
            # Reset collision state
            self.collision_intensity = 0.0
            self.collision_actor = None
            return self.weights.collision_any
            
        return 0.0
    
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
        hero = self.hero_manager.hero
        if hero is None:
            return {}
            
        velocity = hero.get_velocity()
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        return {
            'speed_kmh': speed * 3.6,
            'speed_ms': speed,
            'distance_to_goal': self.previous_distance_to_goal if self.previous_distance_to_goal else -1,
        }
