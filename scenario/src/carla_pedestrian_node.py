#!/usr/bin/env python

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
import rospy
from carla_msgs.msg import CarlaWorldInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

class SpawnWalker(object):
    def __init__(self):
        rospy.init_node("carla_pedestrian_node", anonymous=True)
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', 2000)
        self.timeout = rospy.get_param('/carla/timeout', 10)
        self.world = None
        self.actors = None
        self.player = []
        self.initialpose_subscriber_walker = rospy.Subscriber("/carla/ego_vehicle/initialpose", PoseWithCovarianceStamped, self.reset_walker)
        self.reset_mode = False

        # Read parameters
        self.number = len(rospy.get_param("/agent"))
        self.Spawnpoint = np.zeros((self.number, 2))
        self.Waypoint = np.zeros((self.number, 6, 2))
        self.Speed = np.zeros((self.number, 1))
        self.Threshold = np.zeros((self.number, 1))

        for i in range(self.number):
            str_spawnpoint = "/agent/agent_"+str(i+1)+"/spawnpoint"
            self.Spawnpoint[i] = rospy.get_param(str_spawnpoint)

            str_waypoint = "/agent/agent_"+str(i+1)+"/waypoint"
            self.Waypoint[i] = rospy.get_param(str_waypoint)
            
            str_speed = "/agent/agent_"+str(i+1)+"/speed"
            self.Speed[i] = rospy.get_param(str_speed)
            
            str_threshold = "/agent/agent_"+str(i+1)+"/threshold"
            self.Threshold[i] = rospy.get_param(str_threshold)

        self.Rate = float(rospy.get_param("/Distance_check_rate"))
    
    def on_shutdown(self):
        """
        callback on shutdown
        """
        rospy.loginfo("Shutting down, stopping walker...")
        
    def reset_walker(self, initial_pose):
        """
        Callback for /initialpose
        """
        # option 1: destroy and then respawn
        # option 2: set to the original position and then reset the next waypoint
        for i in range(self.number):
            respawn_point = carla.Location(x=self.Spawnpoint[i][0], y=self.Spawnpoint[i][1])
            self.actors[i].set_location(respawn_point)
        self.reset_mode = True

        # rospy.on_shutdown(self.on_shutdown)
        # self.DestroyActors()
        # self.actors = None
        # self.player = []
        # self.SpawnPedestrians(Spawnpoint)
        # self.StartNWalker(Spawnpoint, Waypoint, Speed, Rate, Threshold, self.number)
        # print("walker_respawned")
    
    def SpawnPedestrians(self):
        """
        (Re)spawns the pedestrians at a given actor spawnpoint
        """
        # Take all the locations to spawn
        spawn_points = []
        for i in range(self.number):
            spawn_point = carla.Transform(carla.Location(x=self.Spawnpoint[i][0], y=self.Spawnpoint[i][1]))

            if (spawn_point.location != None):
                spawn_points.append(spawn_point)
        
        # Spawn the walker object
        self.blueprintsWalkers = self.world.get_blueprint_library().filter("walker.pedestrian.*")
        for spawn_point in spawn_points:
            self.walker_bp = random.choice(self.blueprintsWalkers)
            # set as not invincible
            if self.walker_bp.has_attribute('is_invincible'):
                self.walker_bp.set_attribute('is_invincible', 'true')
            self.player.append(self.world.spawn_actor(self.walker_bp, spawn_point))
        self.actors = self.player
        print("Spawned %d pedestrians" %len(self.actors))

        return self.actors

    def PedestrianTarget(self, actor, CurrentPoint, NextPoint, Speed):

        Control = carla.WalkerControl()
        diff = NextPoint - CurrentPoint
        # print("next point", NextPoint)
        # print("current point", CurrentPoint)
        # print(diff)
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

        return dist

    def StartNWalker(self):
        actors = self.actors
        SpawnLocation = self.Spawnpoint
        Waypoint = self.Waypoint
        Speed = self.Speed
        Rate = self.Rate
        Threshold = self.Threshold

        # Start all the pedestrians
        dist = np.zeros((self.number, 1))
        location_idx = np.zeros((self.number))

        for i in range(self.number):
            TargetLocation = Waypoint[i]
            dist[i] = self.PedestrianTarget(self.actors[i], SpawnLocation[i], TargetLocation[int(location_idx[i])], Speed[i])

        # Distance check for all the pedestrians
        while not rospy.is_shutdown():
            try:
                if self.reset_mode is True:
                    print("Resetting the walker...")
                    for i in range(self.number):
                        location_idx[i] = 0
                        TargetLocation = Waypoint[i]
                        dist[i] = self.PedestrianTarget(self.actors[i], SpawnLocation[i], TargetLocation[int(location_idx[i])], Speed[i])
                    self.reset_mode = False
                
                else:
                    for i in range(self.number):
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
            
            except rospy.ROSInterruptException:
                pass

    def DestroyActors(self):
        if self.actors:
            for i in range(self.number):
                self.actors[i].destroy()
        self.actors = None
        print('\ndestroying %d walkers' % self.number)
    
    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for world info!")
            sys.exit(1)
        
        rospy.loginfo("CARLA world available. Spawn walkers...")
        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()
        self.SpawnPedestrians()
        self.StartNWalker()

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        
def main():
    """
    main function
    """
    ego_walker = SpawnWalker()
    try:
        ego_walker.run()
    finally:
        if ego_walker is not None:
            ego_walker.DestroyActors()

if __name__ == '__main__':
    main()

