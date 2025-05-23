import carla
import gymnasium as gym
import random
import numpy as np
import pygame
from simulation.settings import *



class CarlaEnvironment():
	def __init__(self, client, world, town, checkpoint_frequency=100, continous_action=True) -> None:
		self.client = client
		self.world = world
		self.blueprint_library = self.world.get_blueprint_library()
		self.map = self.world.get_map()
		self.action_space = self.get_discrete_action_space()
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


	def step(self, action_idx):


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
#                                                                     #
#---------------------------------------------------------------------#
