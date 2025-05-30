import time

import carla
from traffic_config import *

from connection_manager import ClientConnection

import argparse
import logging
from numpy import random



class GenerateTraffic:
        def __init__(self):
                self.number_of_vehicles = number_of_vehicles
                self.number_of_walkers = number_of_walkers
                self.hero = hero


                self.vehicles_list = []
                self.walkers_list = []
                self.all_id = []
                conn = ClientConnection(town="Town10HD_Opt")       
                self.client, self.world = conn.connect()
                self.client.set_timeout(10.0)

                self.synchronous_master = False
                random.seed(seed if seed is not None else int(time.time()))
                
                self.traffic_manager = self.client.get_trafficmanager(tm_port)
                self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
                if respawn:
                        self.traffic_manager.set_respawn_dormant_vehicles(True)
                if hybrid:
                        self.traffic_manager.set_hybrid_physics_mode(True)
                        self.traffic_manager.set_hybrid_physics_radius(70.0)
                if seed is not None:
                         self.traffic_manager.set_random_device_seed(seed)

                settings = self.world.get_settings()

                if not asynch:
                        self.traffic_manager.set_synchronous_mode(True)
                        if not settings.synchronous_mode:
                                self.synchronous_master = True
                                settings.synchronous_mode = True
                                settings.fixed_delta_seconds = 0.05
                        else:
                                self.synchronous_master = False
                else:
                        print("You are currently in asynchronous mode, and traffic might experience some issues")
                
                if no_rendering:
                        settings.no_rendering_mode = True
                self.world.apply_settings(settings)

                self.blueprints = self.get_actor_blueprints(self.world, filterv, generationv)

                if not self.blueprints:
                        raise ValueError("Couldn't find any vehicles with the specified filters")
                self.blueprintsWalkers = self.get_actor_blueprints(self.world, filterw, generationw)
                if not self.blueprintsWalkers:
                        raise ValueError("Couldn't find any walkers with the specified filters")

                if safe:
                        self.blueprints = [x for x in self.blueprints if x.get_attribute('base_type') == 'car']

                self.blueprints = sorted(self.blueprints, key=lambda bp: bp.id)

                self.spawn_points = self.world.get_map().get_spawn_points()
                number_of_spawn_points = len(self.spawn_points)

                if self.number_of_vehicles < number_of_spawn_points:
                        random.shuffle(self.spawn_points)
                elif self.number_of_vehicles > number_of_spawn_points:
                        msg = 'requested %d vehicles, but could only find %d spawn points'
                        logging.warning(msg, number_of_vehicles, number_of_spawn_points)
                        self.number_of_vehicles = number_of_spawn_points

                # @todo cannot import these directly.
                self.SpawnActor = carla.command.SpawnActor
                self.SetAutopilot = carla.command.SetAutopilot
                self.FutureActor = carla.command.FutureActor


        def get_actor_blueprints(self, world, filter, generation):
                bps = world.get_blueprint_library().filter(filter)

                if generation.lower() == "all":
                        return bps
                
                if len(bps) == 1:
                        return bps
                
                try:
                        int_generation = int(generation)
                        if int_generation in [1, 2, 3]:
                                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                                return bps
                        else:
                                print("   Warning! Actor Generation is not valid. No actor will be spawned.")
                                return []
                except:
                        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
                        return []
                



        def generate_vehicles(self):
                batch = []
                for n, transform in enumerate(self.spawn_points):
                        if n >= number_of_vehicles:
                                break
                        blueprint = random.choice(self.blueprints)
                        if blueprint.has_attribute('color'):
                                color = random.choice(blueprint.get_attribute('color').recommended_values)
                                blueprint.set_attribute('color', color)
                        if blueprint.has_attribute('driver_id'):
                                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                                blueprint.set_attribute('driver_id', driver_id)
                        if self.hero:
                                blueprint.set_attribute('role_name', 'hero')
                                self.hero = False
                        else:
                                blueprint.set_attribute('role_name', 'autopilot')
                        # spawn the cars and set their autopilot and light state all together
                        batch.append(self.SpawnActor(blueprint, transform)
                                .then(self.SetAutopilot(self.FutureActor, True, self.traffic_manager.get_port())))
                        
                for response in self.client.apply_batch_sync(batch, self.synchronous_master):
                        if response.error:
                                logging.error(response.error)
                        else:
                                self.vehicles_list.append(response.actor_id)
                
                # Set automatic vehicle lights update if specified
                if car_lights_on:
                        all_vehicle_actors = self.world.get_actors(self.vehicles_list)
                        for actor in all_vehicle_actors:
                                self.traffic_manager.update_vehicle_lights(actor, True)

                self.traffic_manager.global_percentage_speed_difference(30.0)

                # while True:
                #         if not asynch and self.synchronous_master:
                #                 self.world.tick()
                #         else:
                #                 self.world.wait_for_tick()
                        
                return
        

        def generate_walkers(self):
                # some settings
                percentagePedestriansRunning = 50.0      # how many pedestrians will run
                percentagePedestriansCrossing = 50.0     # how many pedestrians will walk through the road
                if seedw:
                        self.world.set_pedestrians_seed(seedw)
                        random.seed(seedw)
                # 1. take all the random locations to spawn
                spawn_points = []
                for i in range(number_of_walkers):
                        spawn_point = carla.Transform()
                        loc = self.world.get_random_location_from_navigation()
                        if (loc != None):
                                spawn_point.location = loc
                                #Apply Offset in vertical to avoid collision spawning
                                spawn_point.location.z += 2
                                spawn_points.append(spawn_point)

                # 2. we spawn the walker object
                batch = []
                walker_speed = []

                for spawn_point in spawn_points:
                        walker_bp = random.choice(self.blueprintsWalkers)
                        # set as not invincible
                        if walker_bp.has_attribute('is_invincible'):
                                walker_bp.set_attribute('is_invincible', 'false')
                        # set the max speed
                        if walker_bp.has_attribute('speed'):
                                if (random.random() > percentagePedestriansRunning):
                                        # walking
                                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                                else:
                                        # running
                                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
                        else:
                                print("Walker has no speed")
                                walker_speed.append(0.0)
                        batch.append(self.SpawnActor(walker_bp, spawn_point))
                results = self.client.apply_batch_sync(batch, True)
                walker_speed2 = []
                for i in range(len(results)):
                        if results[i].error:
                                logging.error(results[i].error)
                        else:
                                self.walkers_list.append({"id": results[i].actor_id})
                                walker_speed2.append(walker_speed[i])
                walker_speed = walker_speed2
                # 3. we spawn the walker controller
                batch = []
                walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
                for i in range(len(self.walkers_list)):
                        batch.append(self.SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
                results = self.client.apply_batch_sync(batch, True)
                for i in range(len(results)):
                        if results[i].error:
                                logging.error(results[i].error)
                        else:
                                self.walkers_list[i]["con"] = results[i].actor_id
                # 4. we put together the walkers and controllers id to get the objects from their id
                for i in range(len(self.walkers_list)):
                        self.all_id.append(self.walkers_list[i]["con"])
                        self.all_id.append(self.walkers_list[i]["id"])
                all_actors = self.world.get_actors(self.all_id)

                # wait for a tick to ensure client receives the last transform of the walkers we have just created
                if asynch or not self.synchronous_master:
                        self.world.wait_for_tick()
                else:
                        self.world.tick()
                
                # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
                # set how many pedestrians can cross the road
                self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
                for i in range(0, len(self.all_id), 2):
                        # start walker
                        all_actors[i].start()
                        # set walk to random point
                        all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
                        # max speed
                        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

                print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(self.vehicles_list), len(self.walkers_list)))

                # Example of how to use Traffic Manager parameters
                self.traffic_manager.global_percentage_speed_difference(30.0)

                # while True:
                #         if not asynch and self.synchronous_master:
                #                 self.world.tick()
                #         else:
                #                 self.world.wait_for_tick()


                return
        



        def generate_all(self):
                trafficManager = GenerateTraffic()
                trafficManager.generate_vehicles()
                trafficManager.generate_walkers()
                while True:
                        if not asynch and self.synchronous_master:
                                self.world.tick()
                        else:
                                self.world.wait_for_tick()

                return
        




if __name__ == '__main__':

    try:
        GenerateTraffic().generate_all()
        
        
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
