import carla
import gymnasium as gym
import cv2
import time

from connection_manager import ClientConnection
from sensors.sensor_manager import SensorManager
from sensors.sensor_interface import SensorInterface
from traffic_manager import TrafficManager
from hero_manager import HeroManager

from hero_config import *


class CarlaEnv(gym.Env):
        def __init__(self):
                super().__init__()
                self.conn = ClientConnection(town="Town10HD_Opt")       
                self.client, self.world = self.conn.connect() 
                self.conn.cleanup_all_actors() 
                
                traffic_manager = TrafficManager(self.client, self.world)
                self.sensor_interface = SensorInterface()
                self.sensor_manager = SensorManager()

                self.hero_manager = HeroManager(self.client, self.world, self.sensor_interface, self.sensor_manager)



        def reset(self, *, seed = None, options = None):
                self.hero_manager.reset_hero(hero_config)
                self.hero_manager.set_spectator_camera_view()

                self.hero_manager.hero.set_autopilot(True)      #test

                return
        


        def step(self, action=None):
                self.world.tick()
                return
        
        def get_action_space(self):
                return
        
        def get_observation_space(self):
                return
        




        def render(self):
                self._camera_name = ['camera_front', 'camera_top']
                self.render_mode = "human"
                sensor_data = self.hero_manager.get_sensor_data()
                for name in self._camera_name:
                        if name in sensor_data:
                                frame_id, image_data = sensor_data[name]
                        else:
                                print(f"Invalid image data from {name}")

                        if self.render_mode == "human":
                                bgr_image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)
                                cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
                                
                        cv2.imshow(name, bgr_image)

                        key = cv2.waitKey(1) & 0xFF


                        if key == ord('q'):
                                return
                        
                return
        


        
        def close(self):
                print("[EXITING] Gymnasium: Closing CARLA environment...")

                try:
                        cv2.destroyAllWindows()
                        cv2.waitKey(1)

                        self.conn.cleanup_all_actors()

                        settings = env.world.get_settings()
                        settings.synchronous_mode = False
                        settings.no_rendering_mode = False
                        settings.fixed_delta_seconds = None
                        env.world.apply_settings(settings)

                        time.sleep(0.5) 

                        print("[EXIT SUCCESSFULL]")

                except Exception as e:
                        print(f"[ERROR] Gymnasium Close(): {e}")
                



if __name__ == "__main__":
        env=CarlaEnv()
        env.reset()
        while(True):
                try:
                        env.render()
                        env.step()
                        time.sleep(0.01)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                                env.close()
                                break
                
                except Exception as e:
                        print(f"[ERROR] Gymnasium \"__main__\": {e}")

        




