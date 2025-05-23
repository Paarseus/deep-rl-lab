import carla
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
	def step(self, action_idx):

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
