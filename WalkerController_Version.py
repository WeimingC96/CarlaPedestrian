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
import xml.etree.ElementTree as ET

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

print('Initialization ...')

# Read parameters
tree = ET.parse('crossing_2people.xml')
root = tree.getroot()

Spawnpoint = np.zeros((len(root)-1, 2))
Waypoint = np.zeros((len(root)-1, 6, 2))
Speed = np.zeros((len(root)-1, 1))
Threshold = np.zeros((len(root)-1, 1))

Rate = float(root.find('Distance_check_rate').text)
number = len(root.findall('agent'))

for idx, agent in enumerate(root):
    for idx2, par in enumerate(agent):
        if par.tag == "spawnpoint":
            Spawnpoint[idx][0] = par.attrib.get('x')
            Spawnpoint[idx][1] = par.attrib.get('y')
        elif par.tag == "speed":
            Speed[idx] = par.text
        elif par.tag == "threshold":
            Threshold[idx] = par.text
        elif par.tag == "waypoint":
            Waypoint[idx][idx2-3][0] = par.attrib.get('x')
            Waypoint[idx][idx2-3][1] = par.attrib.get('y')
        else:
            pass

class SpawnWalker:
    def __init__(self):
        self.argparser = argparse.ArgumentParser(description=__doc__)
        self.argparser.add_argument(
            '--host',
            metavar='H',
            default='127.0.0.1',
            help='IP of the host server (default: 127.0.0.1)')
        self.argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        self.argparser.add_argument(
            '-w', '--number-of-walkers',
            metavar='W',
            default=number,
            type=int,
            help='number of walkers (default: 50)')
        self.argparser.add_argument(
            '--filterw',
            metavar='PATTERN',
            default='walker.pedestrian.*',
            help='pedestrians filter (default: "walker.pedestrian.*")')
        self.argparser.add_argument(
            '--tm-port',
            metavar='P',
            default=8000,
            type=int,
            help='port to communicate with TM (default: 8000)')
        self.argparser.add_argument(
            '--sync',
            action='store_true',
            help='Synchronous mode execution')
        self.argparser.add_argument(
            '--hybrid',
            action='store_true',
            help='Enanble')
        self.args = self.argparser.parse_args()
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

        self.client = carla.Client(self.args.host, self.args.port)
        self.client.set_timeout(2.0)
        self.synchronous_master = False

        self.world = self.client.get_world()
        self.blueprintsWalkers = self.world.get_blueprint_library().filter(self.args.filterw)

        if self.args.sync:
            self.settings = self.world.get_settings()
            if not self.settings.synchronous_mode:
                self.synchronous_master = True
                self.settings.synchronous_mode = True
                self.settings.fixed_delta_seconds = 0.05
                self.world.apply_settings(self.settings)
            else:
                self.synchronous_master = False
 
    def SpawnPedestrians(self, SpawnLocation):
        # 1. Take all the locations to spawn
        spawn_points = []
        
        for i in range(self.args.number_of_walkers):
            spawn_point = carla.Transform(carla.Location(x=SpawnLocation[i][0], y=SpawnLocation[i][1]))

            if (spawn_point.location != None):
                spawn_points.append(spawn_point)
        
        # 2. Spawn the walker object
        self.actors = []
        for spawn_point in spawn_points:
            self.walker_bp = random.choice(self.blueprintsWalkers)
            # set as not invincible
            if self.walker_bp.has_attribute('is_invincible'):
                self.walker_bp.set_attribute('is_invincible', 'true')
            self.actors.append(self.world.spawn_actor(self.walker_bp, spawn_point))
        print("Spawned %d pedestrians" %len(self.actors))
       
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()  

        return self.actors       

    def PedestrianTarget(self, actor, CurrentPoint, NextPoint, Speed):

        Control = carla.WalkerControl()
        diff = NextPoint - CurrentPoint
        print("next point", NextPoint)
        print("current point", CurrentPoint)
        print(diff)
        if abs(diff[0]) > abs(diff[1]):
            if diff[0] > 0:
                Control.direction.x=1
                Control.direction.y=0
            else:
                Control.direction.x=-1
                Control.direction.y=0
        else:
            if diff[1] > 0:
                Control.direction.x=0
                Control.direction.y=1
            else:
                Control.direction.x=0
                Control.direction.y=-1
        
        Control.speed = Speed[0]
        actor.apply_control(Control)
        
        current_position = np.array([actor.get_location().x, actor.get_location().y])
        dist = np.linalg.norm(NextPoint - current_position)

        if self.args.sync and self.synchronous_master:
            self.world.tick()
        else:
            self.world.wait_for_tick()
        
        return dist
                
    def StartNWalker(self, SpawnLocation, Waypoint, actors, Speed, Rate, Threshold, number):
        self.actors = actors
        # Start all the pedestrians
        print(len(actors))
        dist = np.zeros((number, 1))
        location_idx = np.zeros((number))

        for i in range(number):
            TargetLocation = Waypoint[i]
            dist[i] = self.PedestrianTarget(self.actors[i], SpawnLocation[i], TargetLocation[int(location_idx[i])], Speed[i])
        print(dist)

        # Distance check for all the pedestrians
        while True:
            for i in range(number):
                if dist[i] > Threshold[i]:
                    TargetLocation = Waypoint[i]
                    print("Pedestrian %d next waypoint:(%d, %d)" %(i+1, TargetLocation[int(location_idx[i])][0], TargetLocation[int(location_idx[i])][1]))
                    print("Pedestrian %d distance to next waypoint is %f" %(i+1, dist[i]))
                    current_position = np.array([actors[i].get_location().x, actors[i].get_location().y])
                    dist[i] = np.linalg.norm(TargetLocation[int(location_idx[i])] - current_position)
                else:
                    location_idx[i] = location_idx[i] + 1
                    TargetLocation = Waypoint[i]
                    if location_idx[i] == TargetLocation.shape[0]:
                        dist[i] = self.PedestrianTarget(self.actors[i], TargetLocation[int(location_idx[i])-1], TargetLocation[0], Speed[i])
                        location_idx[i] = 0
                    else:
                        dist[i] = self.PedestrianTarget(self.actors[i], TargetLocation[int(location_idx[i])-1], TargetLocation[int(location_idx[i])], Speed[i])

            time.sleep(1/Rate)

    def DestroyActors(self, actors):
        self.actors = actors

        for i in range(0, len(actors)):
            actors[i].destroy()

        print('\ndestroying %d walkers' % len(actors))
        
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()

def main():
    try:
        # Run codes only once
        actors = SpawnWalker().SpawnPedestrians(Spawnpoint)
        
        # Start the Controller
        actors = SpawnWalker().StartNWalker(Spawnpoint, Waypoint, actors, Speed, Rate, Threshold, number)
        
    finally:
        SpawnWalker().DestroyActors(actors)
        #pass

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')