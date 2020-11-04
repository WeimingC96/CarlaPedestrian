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

print('Initialization ...')

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
            default=1,
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
        self.client.set_timeout(10.0)
        self.synchronous_master = False

        self.world = self.client.get_world()
        self.blueprintsWalkers = self.world.get_blueprint_library().filter(self.args.filterw)
        
        self.SpawnActor = carla.command.SpawnActor

        if self.args.sync:
            self.settings = self.world.get_settings()
            self.traffic_manager.set_synchronous_mode(True)
            if not self.settings.synchronous_mode:
                self.synchronous_master = True
                self.settings.synchronous_mode = True
                self.settings.fixed_delta_seconds = 0.05
                self.world.apply_settings(self.settings)
            else:
                self.synchronous_master = False

    def Apply_Settings(self):
        self.walkers_list = []
        self.all_id = []
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()
        print('Settings are applied')
        return self.walkers_list, self.all_id

    def SpawnPedestrians(self, walkers_list, all_id, SpawnLocation):
        # 1. Take all the locations to spawn
        self.walkers_list = walkers_list
        self.all_id = all_id
        spawn_points = []
        percentagePedestriansRunning = 0.0

        for i in range(self.args.number_of_walkers):
            spawn_point = carla.Transform()
            spawn_point.location.x = SpawnLocation[0]
            spawn_point.location.y = SpawnLocation[1]
            spawn_point.location.z = SpawnLocation[2]

            if (spawn_point.location != None):
                spawn_points.append(spawn_point)
        
        # 2. Spawn the walker object
        self.batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            print(spawn_point)
            self.walker_bp = random.choice(self.blueprintsWalkers)
            # set as not invincible
            if self.walker_bp.has_attribute('is_invincible'):
                self.walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if self.walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(self.walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(self.walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            self.batch.append(self.SpawnActor(self.walker_bp, spawn_point))
                
        results = self.client.apply_batch_sync(self.batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                #print('Pedestrian %d-th is not spawned' %(i+1))
                #print('Location of that pedestrian is:', spawn_points[i].location)
                logging.error(results[i].error)
            else:
                self.walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        #print(self.walkers_list)
        self.walker_speed = walker_speed2

        # 3. Spawn the walker controller
        self.batch = []
        self.walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
    
        for i in range(len(self.walkers_list)):
            self.batch.append(self.SpawnActor(self.walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
        results = self.client.apply_batch_sync(self.batch, True)
        
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list[i]["con"] = results[i].actor_id

        # 4. Put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(self.walkers_list)):
            self.all_id.append(self.walkers_list[i]["con"])
            self.all_id.append(self.walkers_list[i]["id"])
        self.all_actors = self.world.get_actors(self.all_id)
       
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()  

        return self.walkers_list, self.all_id, self.all_actors, self.walker_speed 
                
    def PedestrianTarget(self, TargetLocation, walkers_list, all_id, all_actors, walker_speed, i): 
        self.walkers_list = walkers_list
        self.all_actors = all_actors
        self.all_id = all_id
        self.walker_speed = walker_speed
        
        # start walker
        self.all_actors[i].start()

        # set walk to random point
        Target = self.world.get_random_location_from_navigation()
        Target.x = TargetLocation[0]
        Target.y = TargetLocation[1]
        Target.z = TargetLocation[2]
        self.all_actors[i].go_to_location(Target)
        
        # max speed
        self.all_actors[i].set_max_speed(float(self.walker_speed[int(i/2)]))    

        #if not self.args.sync or not self.synchronous_master:
            #self.world.wait_for_tick()
        #else:
            #self.world.tick() 
        
        return self.walkers_list, self.all_id, self.all_actors

    def DestroyActors(self, walkers_list, all_id, all_actors):
        # Stop walker controllers
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
        
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()


def main():
    try:
        SpawnLocation = np.array([65, -5, 0])

        TargetLocation = np.array([[80, -5, 0], [80, 5, 0], [65, 5, 0], [80, 5, 0], [80, -5, 0], [65, -5, 0]])

        print(TargetLocation.shape)
        # run codes only once
        walkers_list, all_id = SpawnWalker().Apply_Settings()
        walkers_list, all_id, all_actors, walker_speed = SpawnWalker().SpawnPedestrians(walkers_list, all_id, SpawnLocation)

        # run codes every second
        start_time  = time.time()
        end_time = time.time()
        elapsed_time = end_time- start_time

        # rows are steps, columns are pedestrians
        ReachTarget = np.zeros((TargetLocation.shape[0], 2), dtype = int)

        for i in range(0, len(all_id), 2):
            all_actors[i].start()
        
        while True:
            for i in range(0, len(all_id), 2):
                print(i)
                for j in range(TargetLocation.shape[0]):
                    print(j)
                    print(TargetLocation[j])
                    current_position = np.array([all_actors[i+1].get_location().x, all_actors[i+1].get_location().y, all_actors[i+1].get_location().z])
                    dist = np.linalg.norm(TargetLocation[j] - current_position)
                    #SpawnWalker().PedestrianTarget(TargetLocation[j], walkers_list, all_id, all_actors, walker_speed, i)
                    #Target = world.get_random_location_from_navigation()
                    #Target.x = TargetLocation[j][0]
                    #Target.y = TargetLocation[j][1]
                    #Target.z = TargetLocation[j][2]
                    all_actors[i].go_to_location(carla.Location(x = TargetLocation[j][0], y = TargetLocation[j][1], z = TargetLocation[j][2]))
                    all_actors[i].set_max_speed(3)
                    while dist > 5:
                        current_position = np.array([all_actors[i + 1].get_location().x, all_actors[i + 1].get_location().y, all_actors[i + 1].get_location().z])
                        dist = np.linalg.norm(TargetLocation[j] - current_position)
                        print(dist)
                        time.sleep(.5)
                        if dist > 20:
                            all_actors[i+1].set_location(carla.Location(x = SpawnLocation[0], y = SpawnLocation[1], z = SpawnLocation[2]))

            
                
    finally:
        SpawnWalker().DestroyActors(walkers_list, all_id, all_actors)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')