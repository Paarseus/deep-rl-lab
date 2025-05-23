import os
import sys
import glob
import carla
import random
from settings import PORT, TIMEOUT, HOST, CAR_NAME

class ClientConnection:
    def __init__(self, town):
        self.client = None
        self.town = town

    def setup(self):
        try:

            # Connecting to the  Server
            self.client = carla.Client(HOST, PORT)
            self.client.set_timeout(TIMEOUT)
            self.world = self.client.get_world()
            self.world.set_weather(carla.WeatherParameters.WetSunset)
            return self.client, self.world

        except Exception as e:
            print(
                'Failed to make a connection with the server: {}'.format(e))
            self.error()
    def test_server_with_vehicle(self):
        try:
            blueprint_library = self.world.get_blueprint_library()

            # Find a vehicle blueprint
            vehicle_bp = blueprint_library.filter(CAR_NAME)[0]
            # Pick a spawn point
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                raise RuntimeError("No spawn points available in this map.")
                self.error()
            spawn_point = random.choice(spawn_points)
    
            # Spawn the vehicle
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            print("Vehicle spawned successfully.")

            # Attach the spectator camera to the vehicle
            spectator = self.world.get_spectator()
            transform = carla.Transform(
                spawn_point.location + carla.Location(x=-1, z=1.5),
                spawn_point.rotation
            )
            spectator.set_transform(transform)
            print("Spectator camera set to follow vehicle.")

        except Exception as e:
            print(f"[ERROR] Failed during vehicle spawn test: {e}")
            self.error()

    def error(self):

        print("\nClient version: {}".format(
            self.client.get_client_version()))
        print("Server version: {}\n".format(
            self.client.get_server_version()))

        if self.client.get_client_version() != self.client.get_server_version():
            print(
                "There is a Client and Server version mismatch! Please install or download the right versions.")
                
    def cleanup_all(self):
        print("Cleaning up all vehicles and sensors...")
        actors = self.world.get_actors()
        for actor in actors:
            if actor.type_id.startswith("vehicle.") or actor.type_id.startswith("sensor."):
                try:
                    actor.destroy()
                    print(f"Destroyed: {actor.type_id} (ID: {actor.id})")
                except:
                    print(f"Failed to destroy: {actor.id}")
                
if __name__ == "__main__":
    conn = ClientConnection(town="Town10")       
    conn.setup() 
    conn.cleanup_all() 
    conn.test_server_with_vehicle()  

