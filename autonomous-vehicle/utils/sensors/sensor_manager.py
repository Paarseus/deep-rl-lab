import time

import carla
from sensors.sensor import *

import argparse
import logging
from numpy import random


class SensorManager(object):
    """
    Class to simplify the creation of the different CARLA sensors
    """

    @staticmethod
    def spawn(name, attributes, interface, parent):
        attributes = attributes.copy()
        type_ = attributes.get("type", "")

        if type_ == "sensor.camera.rgb":
            sensor = CameraRGB(name, attributes, interface, parent)
        elif type_ == "sensor.camera.depth":
            sensor = CameraDepth(name, attributes, interface, parent)
        elif type_ == "sensor.camera.semantic_segmentation":
            sensor = CameraSemanticSegmentation(name, attributes, interface, parent)
        elif type_ == "sensor.camera.dvs":
            sensor = CameraDVS(name, attributes, interface, parent)
        elif type_ == "sensor.lidar.ray_cast":
            sensor = Lidar(name, attributes, interface, parent)
        elif type_ == "sensor.lidar.ray_cast_semantic":
            sensor = SemanticLidar(name, attributes, interface, parent)
        elif type_ == "sensor.other.radar":
            sensor = Radar(name, attributes, interface, parent)
        elif type_ == "sensor.other.gnss":
            sensor = Gnss(name, attributes, interface, parent)
        elif type_ == "sensor.other.imu":
            sensor = Imu(name, attributes, interface, parent)
        elif type_ == "sensor.other.lane_invasion":
            sensor = LaneInvasion(name, attributes, interface, parent)
        elif type_ == "sensor.other.collision":
            sensor = Collision(name, attributes, interface, parent)
        elif type_ == "sensor.other.obstacle":
            sensor = Obstacle(name, attributes, interface, parent)
        # elif type_ == "sensor.birdview":  # Pseudosensor
        #     sensor = BirdviewManager(name, attributes, interface, parent)
        else:
            raise RuntimeError("Sensor of type {} not supported".format(type_))

        return sensor
    

