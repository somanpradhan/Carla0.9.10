# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
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

# ==============================================================================
# -- Add PythonAPI for release mode -------------------------------------------
# ==============================================================================

try:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/carla')
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass



import carla
from math import radians, cos, sin, degrees, acos, sqrt
import time

# Changing the speed of the vehicle automatically based on the distance to the target
# This is a simple example of how to control the speed of a vehicle automatically based on the distance to the target
def target_velocity(yaw, pitch, speed, desired_speed):
    if speed < 1.66:
        return None

    if speed < desired_speed:
        desired_speed = speed + 0.3
    elif speed > desired_speed:
        desired_speed = speed - 0.3

    
    
    vx = desired_speed * cos(yaw) * cos(pitch)
    vy = desired_speed * sin (yaw) * cos(pitch)
    vz = desired_speed * sin(pitch)

    if desired_speed == 0:
        vx = 0
        vy = 0
        vz = 0
    
    return carla.Vector3D(vx, vy, vz)




# Change speed of the Vehicle
def change_speed(world, some_state, map, vechicle, desired_speed, last_stop_time):
    """
    Change the speed of the vehicle based on the distance to the target
    :param vechicle: The vehicle to control
    :param desired_speed: The desired speed of the vehicle
    """
    # Get the current velocity of the vehicle
    velocity = vechicle.get_velocity()
    transform = vechicle.get_transform()
    yaw = radians(transform.rotation.yaw)
    pitch = radians(transform.rotation.pitch)
    traffic_light_state = str(vechicle.get_traffic_light_state())
    current_time = time.time()
    if some_state or ((traffic_light_state == "Red" or traffic_light_state == "Yellow") and (is_red_light_ahead(vechicle, world, map))):
        desired_speed = 0

    vehicle_list = world.get_actors().filter("*vehicle*")
    person_list = world.get_actors().filter("*walker*")
    distance = 100
    for v in vehicle_list:
        if desired_speed == 0:
            break
        if v.id != vechicle.id:
            infront, dist = is_something_ahead(vechicle, v)
            if infront and dist < 10.0 and dist < distance:
                velo = v.get_velocity()
                desired_speed = (velo.x**2 + velo.y**2 + velo.z**2)**0.5
                distance = dist
    for p in person_list:
        if desired_speed == 0:
            break
        infront, dist = is_something_ahead(vechicle, p, max_angle=20)

        if infront and dist < 10.0:
            waypoint = p.get_transform().location
            waypoint = map.get_waypoint(waypoint, project_to_road=False)
            if waypoint.lane_type == carla.LaneType.Driving:
                desired_speed = 0
                

    # Get the current speed of the vehicle
    speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

    # Calculate the new velocity based on the desired speed
    new_velocity = target_velocity(yaw, pitch, speed, desired_speed)
    
    # Set the new velocity of the vehicle
    if desired_speed == 0:
        vechicle.set_simulate_physics(False)
        vechicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer = 0))
    elif new_velocity is not None:
        vechicle.set_simulate_physics(True)
        vechicle.set_target_velocity(new_velocity)
    return last_stop_time

# Check if the vehicle is a head
def is_something_ahead(ego_vehicle, target, max_distance=20,  max_angle=90):

    target_location = target.get_transform().location
    ego_transform = ego_vehicle.get_transform()
    ego_location = ego_transform.location
    ego_forward = ego_transform.get_forward_vector()

    ego_forward = normalize_vector(ego_transform.get_forward_vector())
    direction_vector = target_location - ego_location
    direction_vector = normalize_vector(carla.Location(
                        x=target_location.x - ego_location.x,
                        y=target_location.y - ego_location.y,
                        z=target_location.z - ego_location.z))
    # Dot Product
    dot = (ego_forward.x * direction_vector.x +
                ego_forward.y * direction_vector.y +
                ego_forward.z * direction_vector.z)
    distance = ((target_location.x - ego_location.x)**2 + 
                (target_location.y - ego_location.y)**2 + 
                (target_location.z - ego_location.z)**2)**0.5
    if distance > max_distance:
        return False, distance

    # Clamp to avoid numerical errors with acos
    dot = max(min(dot, 1.0), -1.0)
    angle = degrees(acos(dot))
    return angle < max_angle, distance

# Normalize Vector
def normalize_vector(v):
    norm = (v.x**2 + v.y**2 + v.z**2)**0.5  # Correct: Euclidean norm (L2)
    if norm == 0:
        return carla.Vector3D(0, 0, 0)
    return carla.Vector3D(v.x / norm, v.y / norm, v.z / norm)

def is_pedestrian_ahead(egro_vechile, pedestrainLocation, max_distance, angle ):
    #do something

    pass


# Check the red light ahead infornt
def is_red_light_ahead(hero, world, map, max_distance=45.0, min_distance = 20, angle_threshold=25.0):
    """
    Checks if there's a red traffic light in front of the vehicle.
    - max_distance: how far ahead to check (in meters)
    - angle_threshold: how narrow the forward cone is (degrees)
    """
    ego_transform = hero.get_transform()
    ego_location = ego_transform.location
    ego_forward = ego_transform.get_forward_vector()


    traffic_lights = world.get_actors().filter("*traffic_light*")
    
    for tl in traffic_lights:
        tl_location = tl.get_transform().location

        # Vector from ego to traffic light
        direction = tl_location - ego_location
        distance = sqrt(direction.x**2 + direction.y**2)

        if distance > max_distance:
            continue  # too far away
        # Normalize direction
        direction_norm = carla.Vector3D(direction.x / distance, direction.y / distance, 0.0)

        # Normalize ego forward
        forward_norm = carla.Vector3D(ego_forward.x, ego_forward.y, 0.0)
        forward_mag = sqrt(forward_norm.x**2 + forward_norm.y**2)
        forward_norm.x /= forward_mag
        forward_norm.y /= forward_mag

        # Compute angle between forward and direction to traffic light
        dot = forward_norm.x * direction_norm.x + forward_norm.y * direction_norm.y
        dot = max(min(dot, 1.0), -1.0)
        angle = degrees(acos(dot))

        if angle < angle_threshold and (tl.state == carla.TrafficLightState.Red or tl.state == carla.TrafficLightState.Yellow):
            return True  # red light ahead!

    return False  # no red light in front
