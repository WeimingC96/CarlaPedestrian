import numpy as np
import xml.etree.ElementTree as ET

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

i = 1
dist = 23.45
location_idx = np.zeros((number))
print(location_idx[1])
print("Pedestrian %d distance to next waypoint is %f" %((i+1), dist))
print(Spawnpoint)
print(Waypoint)
print(Speed)
print(Threshold)
print(Rate)
print(number)

        
walker_bp = random.choice(self.blueprintsWalkers)
        
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            
            transform = random.choice(self.world.get_map().get_spawn_points())
            
            transform = carla.Transform(carla.Location(x=65, y=-5, z=0), carla.Rotation(pitch=0, yaw=0, roll=0))
            
            pedestrian = self.world.spawn_actor(walker_bp, transform)
            print('created %s' % pedestrian.type_id)

            rate = .3

            control=carla.WalkerControl()
            control.speed = 1.5 # speed in the direction
            control.direction.y=1 # direction
            control.direction.x=0
            control.direction.z=0
            pedestrian.apply_control(control)
            time.sleep(5)
