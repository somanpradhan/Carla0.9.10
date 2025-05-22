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
import random
import time
import numpy as np
import cv2
import math 
from collections import defaultdict

def group_connected_roads(graph):
    visited = set()
    groups = []

    for road_id in graph:
        if road_id not in visited:
            stack = [road_id]
            group = set()

            while stack:
                node = stack.pop()
                if node not in visited:
                    visited.add(node)
                    group.add(node)
                    stack.extend(graph[node] - visited)

            groups.append(group)
    
    return groups


def distance(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

def random_spawnpoints(num):
    spawn_points =carla_map.get_spawn_points()
    spawn_points_list = []
    while len(spawn_points_list) <=10:

        status = True
        spawn = random.choice(spawn_points) if spawn_points else carla.Transform()
        for sp in spawn_points_list:
            if distance(spawn.location, sp.location) < 20:
                status = False
                break
        if not(status):
            continue
        spawn_points_list.append(spawn)

    return spawn_points_list 




# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)  # Set a timeout for the connection

# Retrieve the world and map
world = client.get_world()
carla_map = world.get_map()


blueprint_library = world.get_blueprint_library()
sign_bp = blueprint_library.find("static.prop.speed30")
sign_ab = blueprint_library.find("static.prop.speed60")

waypoints = carla_map.generate_waypoints(1.0)

road_junctions = {}

for wp in waypoints:
    road_id = wp.road_id
    junction = wp.get_junction()
    
    if wp.is_junction:
        junction_location = wp.transform.location 

        # Initialize list if key doesn't exist
        if road_id not in road_junctions.keys():
            road_junctions[road_id] = []

        # Avoid duplicates by checking if the location is already stored
        if not any(junction_location.distance(loc) < 1.0 for loc in road_junctions[road_id]):
            road_junctions[road_id].append(junction_location)

# Print result
for rid, locations in road_junctions.items():
    new_locations = [locations[0]]
    for loc in locations:
        min_dis = 1000
        for loc1 in new_locations:
            if loc1 == loc:
                min_dis = 0
                continue
            dis = distance(loc, loc1)
            min_dis = min(min_dis, dis)
        if min_dis > 30 and loc not in new_locations:
            new_locations.append(loc)
    road_junctions[rid] = new_locations


junction_location_all = list()
for location in road_junctions.values():
    for loc in location:
        junction_location_all.append(loc)


# Take 10 random spwan point

sp_10 = random_spawnpoints(10)
for sp in sp_10:
    min_dis = 1000
    
    for loc in junction_location_all:
        min_dis = min(min_dis, distance(loc, sp.location))

    shoulder_speed = carla_map.get_waypoint(
        sp.location, 
        lane_type = carla.LaneType.Sidewalk)
    sp_rotation = sp.rotation
    sp_location = shoulder_speed.transform.location 
    if (min_dis > 10.0):
        sp_location.z += 1.4
        sp_rotation.yaw +=90
        sp_transform = carla.Transform(sp_location, sp_rotation)
        world.try_spawn_actor(sign_ab, sp_transform)
    else:
        print(30)
        sp_rotation.yaw +=180
        sp_transform = carla.Transform(sp_location, sp_rotation)
        world.try_spawn_actor(sign_bp, sp_transform)



# for rid, locations in road_junctions.items():
#     print(f"Road ID: {rid}")
#     if len(locations)==2:
#         distance
#     for location in locations:
#         # if not(rid==413):
#         #     continue
#         location.z +=3
#         transform = carla.Transform(location, carla.Rotation(roll=90))
#         world.try_spawn_actor(sign_bp, transform)



