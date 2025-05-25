import carla
#import gymnasium as gym
import random
import numpy as np
import pygame
from settings import *



class CarlaEnvironment():
	def __init__(self, client, world, town, checkpoint_frequency=100, continous_action=True) -> None:
		self.client = client
		self.world = world
		self.blueprint_library = self.world.get_blueprint_library()
		self.map = self.world.get_map()
#		self.action_space = self.get_discrete_action_space()
		self.continous_action_space = continous_action
		self.display_on = VISUAL_DISPLAY
		self.vehicle = None
		self.settings = None
		self.current_waypoint_index = 0
		self.checkpoint_waypoint_index = 0
		self.fresh_start = True
		self.checkpoint_frequency = checkpoint_frequency
		self.route_waypoints = None
		self.town = town

		self.camera_obj = None
		self.env_camera_obj = None
		self.collision_obj = None
		self.lane_invasion_obj = None

		self.sensor_list = list()
		self.actor_list = list()
		self.walker_list = list()

		self.create_pedestrians()

	def reset(self):
		try:
			if len(self.actor_list) != 0 or len(self.sensor_list) !=0:
				self.client.apply_batch([carla.command.DestoryActor(x) for x in self.sensor_list])
				self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
				self.sensor_list.clear()
				self.actor_list.clear()
			self.remove_sensors()

			vehicle_bp = self.get_vehicle(CAR_NAME)
			if self.town == "Town10":
				transform = self.map.get_spawn_points()[1]
				self.total_distance = 750
			elif self.town == "Town10HD":
				transform = self.map.get_spawn_points()[1]
				self.total_distance = 780
			else:
				transform = random.choice(self.map.get_spawn_points())
				self.total_distance = 250

			self.vahicle = self.world.try_spawn_actor(vehicle_bp, transform)
			self.actor_list.append(self.vehicle)

			self.camera_obj = CameraSensor(self.vehicle)
			while(len(self.camera_obj.front_camera) == 0):
				time.sleep(0.0001)
			self.image_obs = self.camera_obj.front_camera.pop(-1)
			self.sensor_list.append(self.env_camera_obj.sensor)

			if self.display_on:
				self.env_camera_obj = CameraSensorEnv(self.vehicle)
				self.sensor_list.append(self.env_camera_obj.sensor)

			self.collision_obj = CollisionSensor(self.vehicle)
			self.collision_history = self.collision_obj.collision_data
			self.sensor_list.append(self.collision_obj.sensor)

			self.timesteps = 0
			self.rotation = self.vehicle.get_transform().rotation.yaw
			self.previous_location = self.vehicle.get_location()
			self.distance_traveled = 0.0
			self.center_lane_deviation = 0.0
			self.target_speed = 22
			self.max_speed = 25.0
			self.min_speed = 15.0
			self.max_distance_from_center = 3
			self.throttle = float(0.0)
			self.previous_steer = float(0.0)
			self.velocity = float(0.0)
			slef.angle = float(0.0)
			self.distance_covered = 0.0

			if self.fresh_start:
				self.current_waypoint_index =  0
				self.route_waypoints = list()
				self.waypoint = self.map.get_waypoint(self.vehicle.get_location(), project_to_road = True, lane_type = (carla.LaneType.Driving))
				current_waypoint = self.waypoint
				self.route_waypoints.append(current_waypoint)
				for x in range(self.total_distance):
					if self.town == "Town10":
						if x < 650:
							next_waypoint = current_waypoint.next(1.0)[0]
						else:
							next_waypoint = current_waypoint.next(1.0)[-1]
					else:
						next_waypoint = current_waypoint.next(1.0)[0]
					self.route_waypoints.append(next_waypoint)
					current_waypoint = next_waypoint
			else:
				waypoint = self.route_waypoints[self.checkpoint_waypoint_index % len(self.route.waypoints)]
				trasnform = waypoint.transform
				self.vehicle.set_transform
				self.current_waypoint_index = self.checkpoint_waypoint_index
			self.navigation_obs = np.array([self.throttle, self.veolcity, self.previous_steer, self.distance_from_center, self.angle])
			time.sleep(0.5)
			self.collision_history.clear()
			self.episode_start_time = time.time()
			return [self.image_obs, self.navigation_obs]

		except Exception as e:
			print(f"[ERROR] Reset failed: {e}")
			self.client.apply_batch([carla.command.DestroyActor(x) for x in self.sensor_list])
			self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
			self.client.apply_batch([carla.command.DestroyActor(x) for x in self.walker_list])
			self.sensor_list.clear()
			self.actor_list.clear()
			self.remove_sensors()
			if self.display_on:
				pygame.quit()

#-------------------------------------------------------------------#
#	Step method for implementing actions taken by our agent     #
#-------------------------------------------------------------------#

	def step(self, action_idx):
		try:
			self.timestep += 1
			self.fresh_start = False

			velocity = self.vehicle.get_velocity()
			self.velocity = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6

			if self.continous_action_space:
				steer = float(action_idx[0])
				steer = max(min(steer, 1.0), -1.0)
				throttle = float((action_idx[1] + 1.0)/2)
				throttle = max(min(throttle, 1.0), 0.0)
				self.vehicle.apply_control(carla.VehicleControl(steer=self.previous_steer*9 + steer*0.1, throttle=self.throttle*0.9 + throttle*0.1))
				self.previous_seet = steer
				self.throttle = throttle
			else:
				steer = self.action_space[action_idx]
				if self.velocity < 20.0:
					self.vehicle.apply_control(carla.VehicleControl(steer=self.previous_steer*0.9  + steer*0.1, throttle=1.0))
				else:
					self.vehicle.apply_control(carla.VehicleControl(steer=self.previous_steer*0.9 + steer*0.1))
				self.previous_steer = steer
				self.throttle = 1.0

			if self.vehicle.is_at_traffic_light():
				traffic_light = self.vehicle.get_traffic_light()
				if traffic_light.get_state() == carla.TrafficLightState.Red:
					traffic_light.set_state(carla.TrafficLightState.Green)

			self.collision_history = self.collision_obj.collision_data

			self.rotation = self.vehicle.get_transform().rotation.yaw
			self.location = self.vehicle.get_location()

			waypoint_index = self.current_waypoint_index
			for _ in range(len(self.route_waypoints)):
				next_waypoint_index = waypoint_index + 1
				wp = self.route_waypoints[next_waypoint_index % len(self.route_waypoints)]
				dot = np.dot(self.vector(wp.transform.get_forward_vector())[:2], self.location - wptransform.location)[:2]
				if dot > 0.0:
					waypoint_index += 1
				else:
					break

			self.current_waypoint_index = waypoint_index
			self.current_waypoint = self.route_waypoints[self.current_waypoint_index % len(self.route_waypoints)]
			self.next_waypoint = self.route_waypoints[(self.current_waypoint_index + 1) % len(self.route_waypoints)]
			self.distance_from_center = self.distance_to_line(self.vector(self.current_waypoint.transform.location), self.vector(next_waypoint.transform.location), self.vector(self.location))
			self.center_lane_deviation += self.distance_from_center

			fwd = self.vector(self.vehicle.get_velocity())
			wp_fwd = self.vector(self.current_waypoint.transform.roation.get_forward_vector())
			self.angle = self.angle_diff(fwd, wp_fwd)

			if not self.fresh_start:
				if self.checkpoint_frequency is not None:
					self.checkpoint_waypoint_index = (self.current_waypoint_index // self.checkpoint_frequency) * self.checkpoint_frequency

			done = False
			reward = 0

			if len(self.collision_history) != 0:
				done = True
				reward = -10
			elif self.distance_from_center > self.max_distance_from_center:
				done = True
				reward = -10
			elif self.episode_start_time + 10 < time.time() and self.velocity < 1.0:
				done = True
				reward = -10
			elif self.velocity > self.max_speed:
				done = True
				reward = -10

			centering_factor = max(1.0 - self.distance_from_center / self.mac_distance_from_center, 0.0)
			angle_factor = max(1.0 - abs(self.angle / np.deg2rad(20)), 0.0)

			if not done:
				if self.continous_action_space:
					if self.velocity < self.min_speed:
						reward = (self.velocity / self.min_speed) * centering_factor * angle_factor
					elif self.velocity > self.target_speed:
						reward = (1.0 - (self.velocity - self.target_speed) / self.max_speed - self.target_speed) * centering_factor * angle_factor
					else:
						reward = 1.0 * centering_factor * angle_factor
				else:
					reward = 1.0 * centering_factor * angle_factor

			if self.timsteps >= 7500:
				done = True
			elif self.current_waypoint_index >= len(self.route_waypoints) - 2:
				done = True
				self.fresh_start = True
				if self.checkpoint_frequency is not None:
					if self.checkpoint_frequency < self.total_distance//2:
						self.checkpoint_frequency += 2
					else:
						self.checkpoint_frequency = None
						self.checkpoint_waypoint_index = 0
			while(len(self.camera_obj.front_camera) == 0):
				time.sleep(0.0001)

			self.image_obs = self.camera_obj.front_camera.pop(-1)
			normalized_velocity = self.velocity/self.target_speed
			normalized_distance_from_center = self.distance_from_center / self.max_distance_from_center
			normalized_angle = abs(self.angle / np.deg2rad(20))
			self.navigation_obs = np.array([self.throttle, self.velocity, normalized_velocity, normalized_distance_from_center, normalized_angle])

			if done:
				self.center_lane_deviation = self.center_lane_deviation / self.timesteps
				self.distance_covered = abs(self.current_waypoint_index - self.checkpoint_waypoint_index)

				for sensor in self.sensor_list:
					sensor.destory()
				self.remove_sensors()
				for actor in self.actor_list:
					actor.destroy()

			return [self.image_obs, self.navigation_obs], reward, done, [self.distance_covered, self.center_lane_deviation]

		except:
			self.client.apply_batch([carla.command.DestoryActor(x) for x in self. sensor_list])
			self.client.apply_batch([carla.command.DestroyActor(X) for x in self.actor_list])
			self.client.apply_batch([carla.command.DestroyActor(x) for x in self.walker_list])
			self.sensor_list.clear()
			self.actor_list.clear()
			self.remove_sensors()
			if self.display_on:
				pygame.quit()


#-------------------------------------------------------------------#
#                           NPC GENERATION                          #
#-------------------------------------------------------------------#

	def create_pedestrians(self):
		try:

			walker_spawn_points = []
			for i in range(NUMBER_OF_PEDESTRIANS):
				spawn_points_ = carla.transform()
				loc = self.world.get_random_location_from_navigation()
				if (loc != None):
					spawn_point_.location = loc
					walker_spawn_points.append(spawn_point_)

			for spawn_point_ in walker_spawn_points:
				walker_bp = random.choice(self.blueprint_library.filter('walker.pedestrian.*'))
				walker_controller_bp = self.blueprint_library.find('controller.ai.walker')
				if walker_bp.has_attribute('is_invicible'):
					walker_bp.set_attribute('is_invicible', 'false')
				if walker_bp.has_attribute('speed'):
					walker_bp.set_attribute('speed', (walker_bp.get_attribute('speed').recommended_values[1]))
				else:
					walker_bp.set_attribute('speed', 0.0)
				walker = self.world.try_spawn_actor(walker_bp, spawn_point_)
				if walker is not None:
					walker_controller = self.world.spawn_actor(walker_controller_bp, carla.Transform(), walker)
					self.walker_list.append(walker_controller.id)
					self.walker_list.append(walker.id)
				all_actors = self.world.get_actors(self.walker_list)

			for i in range(0, len(self.walker_list), 2):
				all_actors[i].start()
				all_actors[i].go_to_location(self.world.get_random_location_from_navigation())

		except:
			self.client.apply_batch([carla.command.DestroyActor(x) for x in self.walker_list])

	def set_other_vehicles(self):
		try:
			for _ in range(0, NUMBER_OF_VEHICLES):
				spawn_point = random.choice(self.map.get_spawn_points())
				bp_vehicle = random.choice(self.blueprint_library.filter('vehilce'))
				other_vehicle = self.world.try_spawn_actor(bp_vehicle, spawn_point)
			if other_vehicle is not None:
				other_vehicle.set_autopilot(True)
				self.actor_list.append(other_vehicle)
			print("Traffic has been generated with autopilot mode.")
		except:
			self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])

#---------------------------------------------------------------------#
#				EXTRAS                                                                     #
#---------------------------------------------------------------------#

	def change_town(self, new_town):
		self.world = self.client.load_world(new_town)

	def get_world(self) -> object:
		return self.world

	def get_blueprint_library(self) -> object:
		return self.world.get_blueprint_library()

	def angle_diff(self, v0, v1):
		angle = np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0])
		if angle > np.pi: angle -= 2 * np.pi
		elif angle <= -np.pi: angle += 2 * np.pi
		return angle

	def distance_to_line(self, A, B, p):
		num = np.linelg.norm(np.cross(B - A, A - p))
		denom = np.linalg.norm(B - A)
		if np.isclose(denom, 0):
			return np.linalg.norm(p - A)
		return num / denom

	def vector(self, v):
		if isinstance(v, carla.Location) or isinstance(v, carla.Vector3D):
			return np.array([v.x, v.y, v.z])
		elif isintance(v, carla.Rotation):
			return np.array([v.pitch, v.yaw, v.roll])

	def get_discrete_action_space(self):
		action_space = \
			np.array([
			-0.50,
			-0.30,
			-0.10,
			0.0,
			0.10,
			0.30,
			0.50])
		return action_space

	def get_vehicle(self, vehicle_name):
		blueprint = self.blueprint_library.filter(vehicle_name)[0]
		if blueprint.has_attribute('color'):
			color = random.choice(blueprint.get_attribute('color').recommended_values)
			blueprint.set_attribute('color', color)
		return blueprint

	def set_vehicle(Set, vehicle_bp, spawn_points):
		spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
		self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

	def remove_sensors(self):
		self.camera_obj = None
		self.collision_obj = None
		self.lane_invasion_obj = None
		self.env_camera_obj = None
		self.from_camera = None
		self.collision_history = None
		self.wrong_maneuver = None
