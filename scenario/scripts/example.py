## Modifeid by Kasra Mokhtari, Apr 26th, features:  (run this pls: python spawn_npc_Kasra_FV_1.py -w 1)
# 1) has class for spawning vehicles and pedestrians
# 2) Pedesrian control function is separated from pedestrians spawning function so I can update target location
# 3) check the pedestrian location to target location and will stop pedestrian when it hits the target
#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import time
import numpy as np
import argparse
import logging
import random
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

start = time.time()
print('Initialization ...')

class SpawningTask:
    def __init__(self):
        self.argparser = argparse.ArgumentParser(
        description=__doc__)
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
            '-n', '--number-of-vehicles',
            metavar='N',
            default=10,
            type=int,
            help='number of vehicles (default: 10)')
        self.argparser.add_argument(
            '-w', '--number-of-walkers',
            metavar='W',
            default=8,
            type=int,
            help='number of walkers (default: 50)')
        self.argparser.add_argument(
            '--safe',
            action='store_true',
            help='avoid spawning vehicles prone to accidents')
        self.argparser.add_argument(
            '--filterv',
            metavar='PATTERN',
            default='vehicle.*',
            help='vehicles filter (default: "vehicle.*")')
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

        # @todo cannot import these directly.
        self.SpawnActor = carla.command.SpawnActor
        
        if self.args.sync:
            self.settings = self.world.get_settings()
            if not self.settings.synchronous_mode:
                self.synchronous_master = True
                self.settings.synchronous_mode = True
                self.settings.fixed_delta_seconds = 0.05
                self.world.apply_settings(self.settings)
            else:
                self.synchronous_master = False

    def Apply_Settings_Kasra(self):
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()
        print('Settings are applied!')
        return self.vehicles_list, self.walkers_list, self.all_id
        
    def SpawnVehicles(self, vehicles_list):   
        # --------------
        # Spawn vehicles
        # --------------
        self.batch = []
        self.vehicles_list = vehicles_list
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.number_of_spawn_points = len(self.spawn_points)
        for n, transform in enumerate(self.spawn_points):
            if n >= self.args.number_of_vehicles:
                break
            blueprint = random.choice(self.blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            self.batch.append(self.SpawnActor(blueprint, transform).then(self.SetAutopilot(self.FutureActor, True, self.traffic_manager.get_port())))

        for response in self.client.apply_batch_sync(self.batch, self.synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                self.vehicles_list.append(response.actor_id)
        self.world.tick()                
        print('Vehicles are spawned!')

    def SpawnWalker(self, walkers_list, all_id, vehicles_list, Pedestrian_Locations_Read_From_Map):
        self.walkers_list = walkers_list
        self.all_id = all_id
        percentagePedestriansRunning = 0.0      # percentage pedestrians will run
        percentagePedestriansCrossing = 0.0
        spawn_points = []
        s = (1,3)
        Pedestrian_Location_Read_From_Map = np.zeros(s)
        epsilon = 0.5

        print(Pedestrian_Locations_Read_From_Map.shape)
        print(self.args.number_of_walkers)
        for i in range(self.args.number_of_walkers):
            spawn_point = carla.Transform()
            spawn_point.location.x = Pedestrian_Locations_Read_From_Map[i][0]
            spawn_point.location.y = Pedestrian_Locations_Read_From_Map[i][1]
            spawn_point.location.z = Pedestrian_Locations_Read_From_Map[i][2]
            #print(loc)
            if (spawn_point.location != None):
                spawn_points.append(spawn_point)

        # 2. we spawn the walker object
        self.batch = []    
        walker_speed = []
        for spawn_point in spawn_points:
            self.walker_bp = random.choice([self.blueprintsWalkers[4], self.blueprintsWalkers[6], self.blueprintsWalkers[7]])
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
                print('Pedestrian %d-th is not spawned' %(i))
                print('Location of that pedestrian is:', (10**2)*spawn_points[i].location)
                #logging.error(results[i].error)
            else:
                self.walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        #print(self.walkers_list)
        self.walker_speed = walker_speed2

        # 3. we spawn the walker controller
        self.batch = []
        self.walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        #print(walker_controller_bp)
        for i in range(len(self.walkers_list)):
            #print( walkers_list[i]["id"])
            self.batch.append(self.SpawnActor(self.walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
            #print('Batch2', batch)

        results = self.client.apply_batch_sync(self.batch, True)
        for i in range(len(results)):
            #print(results[i].error)
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list[i]["con"] = results[i].actor_id

        #print(walkers_list)
        #  
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(self.walkers_list)):
            self.all_id.append(self.walkers_list[i]["con"])
            self.all_id.append(self.walkers_list[i]["id"])
        self.all_actors = self.world.get_actors(self.all_id)
        #print(all_actors)
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
    
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()  
        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(self.walkers_list)))       
        return vehicles_list, self.walkers_list, self.all_id, self.all_actors, self.walker_speed 

    def PedestrianTarget(self, ReachTarget, Target_Locations, vehicles_list, walkers_list, all_id, all_actors, walker_speed, Step_Matrix_new, i ): 
        percentagePedestriansCrossing = 0.0
        self.walkers_list = walkers_list
        self.all_actors = all_actors
        self.all_id = all_id
        self.walker_speed = walker_speed
        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        if ReachTarget[Step_Matrix_new[int(i/2)]][int(i/2)] == 0:
            #print(all_id[int(i/2)*2])
            #print('Controller is still working')

            # start walker
            self.all_actors[i].start()
            # set walk to random point
            Target_1 = self.world.get_random_location_from_navigation()
            Target_1.x = Target_Locations[int(i/2)][0]
            Target_1.y = Target_Locations[int(i/2)][1]
            Target_1.z = Target_Locations[int(i/2)][2]
            Target = Target_1
            self.all_actors[i].go_to_location(Target)
            # max speed
            #print('Pedestrian Moving velocity: ', self.walker_speed[int(i/2)])
            self.all_actors[i].set_max_speed(float(self.walker_speed[int(i/2)]))    

       
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick() 
        
        return vehicles_list, self.walkers_list, self.all_id, self.all_actors    

    def GetPedestrianLocation(self, ReachTarget, Target_Location, Threshold,  all_id, Step_Matrix, Stop_Ped_count):
        id = all_id
        actor_list = self.world.get_actors()
        distance_tot = []
        PedestrianLocation_matrix_tot = []
        Step_Matrix_new = [Step_Matrix[0], Step_Matrix[1]]
        # Find an actor by id.
        for i in range(0, len(all_id), 2):
            actor = actor_list.find(id[i])
            PedestrianLocation = actor.get_location() 
            PedestrianLocation_matrix = np.array([[PedestrianLocation.x, PedestrianLocation.y, Target_Location[0][2]]])
            Target_Location_m = np.array(Target_Location[int(i/2)][:])
            x = (Target_Location_m[0],Target_Location_m[1]) 
            y = (PedestrianLocation_matrix[0][0], PedestrianLocation_matrix[0][1])
            distance_target_Single = math.sqrt(sum([(a - b) ** 2 for a, b in zip(x, y)]))
            if  Stop_Ped_count[int(i/2)] != 1: 
                print('Pedestrian number = %d Distance to the target number %d is: %f' %( int(i/2)+1, Step_Matrix[int(i/2)]+1, distance_target_Single))   
                if distance_target_Single < Threshold:
                    ReachTarget[Step_Matrix[int(i/2)]][int(i/2)] = 1
                    print('Yohoo! Pedestrian number = %d arrived at the target number %d' %( int(i/2)+1, Step_Matrix[int(i/2)]+1))
                    if Step_Matrix[int(i/2)] != ReachTarget.shape[0]-1:
                        Step_Matrix_new [int(i/2)] += 1
                           
            distance_tot.append(distance_target_Single)
            PedestrianLocation_matrix_tot.append(PedestrianLocation_matrix)

        return ReachTarget, Step_Matrix_new, distance_tot
    
    def StopPedestrian(self, ReachTarget, walkers_list, all_id, all_actors, walker_speed, Target_Locations, i):
                
        percentagePedestriansCrossing = 0.0
        self.walkers_list = walkers_list
        self.all_actors = all_actors
        self.all_id = all_id
        self.walker_speed = walker_speed
        walker_speed[int(i/2)] = 0.0
        
        percentagePedestriansCrossing = 0.0
        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        self.all_actors[i].start()
        # set walk to random point
        Target = self.world.get_random_location_from_navigation()
        Target_1 = self.world.get_random_location_from_navigation()
        Target_1.x = Target_Locations[int(i/2)][0]
        Target_1.y = Target_Locations[int(i/2)][1]
        Target_1.z = Target_Locations[int(i/2)][2]
        Target = Target_1
        #print('ACTORS:', all_actors[i])
        self.all_actors[i].go_to_location(Target)
        # max speed
        #print('Pedestrian Arrived velocity: ', self.walker_speed[int(i/2)])
        self.all_actors[i].set_max_speed(float(self.walker_speed[int(i/2)]))
           
    def DestroyActors(self, vehicles_list, walkers_list, all_id, all_actors):

        print('\ndestroying %d vehicles' % len(vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
        
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()

    def reset_taget(self, distant_matrix, ReachTarget, Target_Locations, vehicles_list, walkers_list, all_id, all_actors, walker_speed, Step_Matrix_new, i):
        
        if distant_matrix[len(distant_matrix)-2][int(i/2)] < distant_matrix[len(distant_matrix)-1][int(i/2)]: 
            #if distant_matrix[len(distant_matrix)-3][int(i/2)] < distant_matrix[len(distant_matrix)-2][int(i/2)]: 
                #if distant_matrix[len(distant_matrix)-4][int(i/2)] < distant_matrix[len(distant_matrix)-3][int(i/2)]:
            print('Reseting Target')
            percentagePedestriansCrossing = 0.0
            self.walkers_list = walkers_list
            self.all_actors = all_actors
            self.all_id = all_id
            self.walker_speed = walker_speed
            self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
            if ReachTarget[Step_Matrix_new[int(i/2)]][int(i/2)] == 0:
                #print(all_id[int(i/2)*2])
                #print('Controller is still working')
                # start walker
                self.all_actors[i].start()
                # set walk to random point
                Target_1 = self.world.get_random_location_from_navigation()
                Target_1.x = Target_Locations[int(i/2)][0]
                Target_1.y = Target_Locations[int(i/2)][1]
                Target_1.z = Target_Locations[int(i/2)][2]
                Target = Target_1
                self.all_actors[i].go_to_location(Target)
                # max speed
                #print('Pedestrian Moving velocity: ', self.walker_speed[int(i/2)])
                self.all_actors[i].set_max_speed(float(self.walker_speed[int(i/2)]))    
                #time.sleep(2)
        
        if not self.args.sync or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick() 
            


def main():
    
    #Pedestrian_Locations_Read_From_Map = np.array([[3658.55835,1179.834106, 80.0],[1500.087891,2257.480469,80.0]])
    #Pedestrian_Locations_Read_From_Map = (10**-2)*(Pedestrian_Locations_Read_From_Map)    # convert cm to m
    #Threshold = 2
    #Target_Locations = [[2828.275146,1178.821289,80.0],[1578.234131, 2143.412598, 80.0]]
    #Target_Locations = (10**-2)*np.asarray(Target_Locations)
    
    Pedestrian_Locations_Read_From_Map = np.array([[-3614.440674, -12716.65625, 80.0],[-2829.476318, -12617.676758, 80.0]])
    Pedestrian_Locations_Read_From_Map = (10**-2)*(Pedestrian_Locations_Read_From_Map)    # convert cm to m
    Threshold = 2

    Target_Locations = [[[-3518.187988, -12011.365234, 80.0], [-2617.028564, -11882.84082, 80.0]], [[-3853.309082, -11973.505859, 80.0], [-2404.904297, -12367.091797, 80.0]], [[-4046.352539, -12290.308594, 80.0], [-1957.000366, -12512.319336, 80.0]]]
    Target_Locations = (10**-2)*np.asarray(Target_Locations)
    
    Pedestrian_Locations_Read_From_Map_tot = Pedestrian_Locations_Read_From_Map
    for i in range(Target_Locations.shape[0]):
        for j in range(Target_Locations.shape[1]):
            Pedestrian_Locations_Read_From_Map_tot = np.concatenate([Pedestrian_Locations_Read_From_Map_tot,np.array([Target_Locations[i][j][:]])])
    
    # run codes only once
    vehicles_list, walkers_list, all_id = SpawningTask().Apply_Settings_Kasra()
    SpawningTask().SpawnVehicles(vehicles_list)
    vehicles_list, walkers_list, all_id, all_actors, walker_speed = SpawningTask().SpawnWalker(walkers_list, all_id, vehicles_list, Pedestrian_Locations_Read_From_Map_tot)
    
    # run codes every second
    ReachTarget_tot = False
    Start_time  = time.time()
    end_time = time.time()
    elapsed_time = end_time- Start_time
    
    ReachTarget = np.zeros((Target_Locations.shape[0], 2), dtype = int)  # rows are steps, columns are pedestrians
    Step_Matrix = [0 for x  in range(0, len(all_id), 2)]
    Step_Matrix_new  = Step_Matrix
    Stop_Ped_count = [0,0]
    Target_Locations_Step_1 = Target_Locations[Step_Matrix[0]][0][:]
    Target_Locations_Step_2 = Target_Locations[Step_Matrix[1]][1][:]
    distant_matrix = []    
    Target_Locations_Step = np.array([Target_Locations_Step_1,Target_Locations_Step_2])
    Target_Locations_Final = Target_Locations_Step
    
    for i in range(0, len(all_id), 2):
        print('Set the target number  = %d for pedestrian %d' %(Step_Matrix[int(i/2)]+1, int(i/2)+1))
        SpawningTask().PedestrianTarget(ReachTarget, Target_Locations_Final, vehicles_list, walkers_list, all_id, all_actors, walker_speed, Step_Matrix, i) # it takes 1.7 seconds to send the command to simulation in each loop (1.7sec* 1.8 m/s  = no control)
 
    while elapsed_time < 90 and ReachTarget_tot == False:
        #print(Step_Matrix)
        
        ReachTarget, Step_Matrix_new, distance_tot = SpawningTask().GetPedestrianLocation(ReachTarget, Target_Locations_Final, Threshold,  all_id, Step_Matrix, Stop_Ped_count)
        #print(ReachTarget)

        distant_matrix.append(distance_tot)
        
        for i in range(0, len(all_id), 2):
            #if Stop_Ped_count[0] == 1 or Stop_Ped_count[1] == 1 :
            Target_Locations_Step_1 = Target_Locations[Step_Matrix_new[0]][0][:]
            Target_Locations_Step_2 = Target_Locations[Step_Matrix_new[1]][1][:]
            Target_Locations_Step = np.array([Target_Locations_Step_1,Target_Locations_Step_2])
            Target_Locations_Final = Target_Locations_Step
            #print(Target_Locations_Final[int(i/2)][:])
            SpawningTask().reset_taget(distant_matrix, ReachTarget, Target_Locations_Final, vehicles_list, walkers_list, all_id, all_actors, walker_speed, Step_Matrix_new, i) 
            
        for i in range(0, len(all_id), 2):
            if Step_Matrix_new[int(i/2)] != Step_Matrix[int(i/2)]:
                print('Step_Matrix = ', Step_Matrix_new)
                print('Set the target number  = %d for pedestrian %d' %(Step_Matrix_new[int(i/2)]+1, int(i/2)+1))
                #set a new Target
                Target_Locations_Step_1 = Target_Locations[Step_Matrix_new[0]][0][:]
                Target_Locations_Step_2 = Target_Locations[Step_Matrix_new[1]][1][:]
                Target_Locations_Step = np.array([Target_Locations_Step_1,Target_Locations_Step_2])
                Target_Locations_Final = Target_Locations_Step
                SpawningTask().PedestrianTarget(ReachTarget, Target_Locations_Final, vehicles_list, walkers_list, all_id, all_actors, walker_speed, Step_Matrix_new, i) # it takes 1.7 seconds to send the command to simulation in each loop (1.7sec* 1.8 m/s  = no control)
            else:
                pass

        #print(Target_Locations_Final[0][:])

        for i in range(0, len(all_id), 2):
            if ReachTarget[ReachTarget.shape[0]-1][int(i/2)]== 1 and Stop_Ped_count [int(i/2)]==0:
                Stop_Ped_count[int(i/2)] += 1
                SpawningTask().StopPedestrian(ReachTarget, walkers_list, all_id, all_actors, walker_speed, Target_Locations_Final, i)
                print('pedestrian %d is set to stop ' %(int(i/2)+1))

        end_time = time.time()
        elapsed_time = end_time- Start_time
        for i in range (0, ReachTarget.shape[0]):
            ReachTarget_tot = (all(X==1 for X in ReachTarget [i][:]))
            
        if ReachTarget_tot == True:
            print('Pedestrian Navigation Task is done = ', ReachTarget_tot)
            print('Time to get there is %f seconds:' %(elapsed_time))
        
        #print(ReachTarget_tot)
        Step_Matrix = Step_Matrix_new

    time.sleep(5)    
    SpawningTask().DestroyActors(vehicles_list, walkers_list, all_id, all_actors)

if __name__ == '__main__':
    main() 
