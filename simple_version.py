"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import csv
import argparse
import logging
import numpy as np
import random
import time
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def carla_initialize():
        argparser = argparse.ArgumentParser(description=__doc__)
        argparser.add_argument(
            '--host',
            metavar='H',
            default='127.0.0.1',
            help='IP of the host server (default: 127.0.0.1)')
        argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        argparser.add_argument(
            '-w', '--number-of-walkers',
            metavar='W',
            default=50,
            type=int,
            help='number of walkers (default: 50)')
        argparser.add_argument(
            '--filterw',
            metavar='PATTERN',
            default='walker.pedestrian.*',
            help='pedestrians filter (default: "walker.pedestrian.*")')
        argparser.add_argument(
            '--tm-port',
            metavar='P',
            default=8000,
            type=int,
            help='port to communicate with TM (default: 8000)')
        argparser.add_argument(
            '--sync',
            action='store_true',
            help='Synchronous mode execution')
        argparser.add_argument(
            '--hybrid',
            action='store_true',
            help='Enanble')
        args = argparser.parse_args()
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
        return args

def spawn_pedestrians():

    args = carla_initialize()

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    
    try:
        world = client.get_world()
        blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)
        walker_bp = random.choice(blueprintsWalkers)
        
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
            
        transform = random.choice(world.get_map().get_spawn_points())
        print(transform)
            
        transform = carla.Transform(carla.Location(x=65, y=-5, z=0), carla.Rotation(pitch=0, yaw=0, roll=0))
        print(transform)
            
        pedestrian = world.spawn_actor(walker_bp, transform)
        print('created %s' % pedestrian.type_id)

        rate = .3

        # The AIController
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        pedestrian_controller = world.spawn_actor(walker_controller_bp, transform, attach_to = pedestrian)
        pedestrian_controller.start()

        pedestrian_controller.go_to_location(carla.Location(x=70, y=-5, z=0))
        while pedestrian.get_location().x < 70:
            print(pedestrian.get_location())
            time.sleep(rate)

        pedestrian_controller.go_to_location(carla.Location(x=70, y=5, z=0))
        while pedestrian.get_location().y < 5:
            print(pedestrian.get_location())
            time.sleep(rate)

        pedestrian_controller.go_to_location(carla.Location(x=65, y=5, z=0))
        while pedestrian.get_location().x > 70:
            print(pedestrian.get_location())
            time.sleep(rate)
            
        pedestrian_controller.go_to_location(carla.Location(x=70, y=5, z=0))
        while pedestrian.get_location().x < 70:
            print(pedestrian.get_location())
            time.sleep(rate)

        pedestrian_controller.go_to_location(carla.Location(x=70, y=-5, z=0))
        while pedestrian.get_location().y > (-5):
            print(pedestrian.get_location())
            time.sleep(rate)

        pedestrian_controller.go_to_location(carla.Location(x=65, y=-5, z=0))
        while pedestrian.get_location().x > 65:
            print(pedestrian.get_location())
            time.sleep(rate)

        while True:
            if args.sync and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
                #pedestrian_controller.go_to_location(carla.Location(x=30, y=-5, z=0))
                #time.sleep(5)

                #pedestrian_controller.set_max_speed(float(walker_speed[int(i/2)]))
                #all_actors[i].go_to_location(world.get_random_location_from_navigation())
                #control=carla.WalkerControl()
                #control.speed = 0.2 # speed in the direction
                #control.direction.y=0 # direction
                #control.direction.x=10
                #control.direction.z=0
                #pedestrian.apply_control(control)
                #time.sleep(5) # how much time the walker will walk in this direction

            #time.sleep(5)
                
    finally:
        pedestrian_controller.stop()
        pedestrian.destroy()
        time.sleep(0.5)

if __name__ == '__main__':
    
    try:
        spawn_pedestrians()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')