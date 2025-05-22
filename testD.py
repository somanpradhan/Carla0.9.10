
connected_roads = defaultdict(set)

for wp in waypoints:
    next_wps = wp.next(1.0)
    for next_wp in next_wps:
        if wp.road_id != next_wp.road_id:
            # Only connect roads that are continuous
            angle_diff = abs(wp.transform.rotation.yaw - next_wp.transform.rotation.yaw) % 360
            if angle_diff < 10 or angle_diff > 350:  # almost straight
                connected_roads[wp.road_id].add(next_wp.road_id)
                connected_roads[next_wp.road_id].add(wp.road_id)


road_groups = group_connected_roads(connected_roads)

print(len(road_groups))