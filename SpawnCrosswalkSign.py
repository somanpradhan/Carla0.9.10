import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from math import atan2, degrees, sqrt

def findAngle(crosswalk_start, crosswalk_end):
# Get the yaw (rotation around Z-axis)
    x = crosswalk_start.x - crosswalk_end.x
    y = crosswalk_start.y - crosswalk_end.y
    return  degrees(atan2(y,x))-90

def adjust_axis(crosswalk):
    crosswalk.y -=1.5
    crosswalk.z +=3
    return crosswalk

def check_crosswalk(crosswalk, implemented_crosswalk):
    count = 0
    index = 0
    while(index < len(implemented_crosswalk)):
        dx = crosswalk.x - implemented_crosswalk[index].x
        dy = crosswalk.y - implemented_crosswalk[index].y
        dis = sqrt(dx**2 + dy**2)
        index+=1
        if dis <= 8:
            return True
    return False


# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)  # Set a timeout for the connection

# Retrieve the world and map
world = client.get_world()
carla_map = world.get_map()

blueprint_library = world.get_blueprint_library()
street_sign = blueprint_library.find("static.prop.crosswalk")
# Get all crosswalk zones
crosswalk_zones = carla_map.get_crosswalks()
index = 0
implemented_crosswalk = []
implemented_yaw = []
while(index < len(crosswalk_zones)):
    if (index+1) % 5 == 0:
        index+=1
        continue
    shoulder_crosswalk = carla_map.get_waypoint(
        crosswalk_zones[index], 
        lane_type = carla.LaneType.Sidewalk)
    yaw_angle = findAngle(crosswalk_zones[index], crosswalk_zones[index+1])
    if shoulder_crosswalk:

        spawn_loc = shoulder_crosswalk.transform.location
        index+=2
        if check_crosswalk(spawn_loc, implemented_crosswalk):
            continue
        
        rotation = carla.Rotation(
            pitch=shoulder_crosswalk.transform.rotation.pitch,
            yaw =yaw_angle ,
            roll=90  # Upright!
            )
        spawn_loc.z +=3
        world.try_spawn_actor(street_sign, carla.Transform(spawn_loc, rotation))
        implemented_crosswalk.append(spawn_loc)