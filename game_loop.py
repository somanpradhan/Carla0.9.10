def game_loop(args):
    """ Main loop for the game """

    pygame.init()
    pygame.font.init() 
    world = None
    tot_target_reached = 0
    num_min_waypoints = 21
    tm_port = 9000
    desired_speed = 4.16

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        #world = World(client.load_world('TownTest01'), args)
        controller = KeyboardControl(world)
        status = True
        
        agent = SelfDrivingAgent(world.player)

        spawn_points = world.map.get_spawn_points()

        if spawn_points[0].location != agent.vehicle.get_location():
            destination = spawn_points[0].location
        else:
            destination = spawn_points[1].location
        destination = spawn_points[75].location
        agent.set_destination(agent.vehicle.get_location(), destination, clean=True)

        time.sleep(3)
        clock = pygame.time.Clock()
        print("Starting simulation")
        last_stop_time = time.time()
        speed_limit = 20

        while True:
            if controller.parse_events():
                return

            if not world.world.wait_for_tick(10.0):
                continue

            world.world.wait_for_tick(10.0)

            labels = world.render(display)
            pygame.display.flip()

            if labels is None:
                pass
            else:
                print("Detected objects: ", labels)
            

            if(labels is None):
                pass
            elif("crosswalk" in labels or "school-sign" in labels):
                speed_limit = 20
            elif ("speed-60" in labels):
                speed_limit = 60
            elif "speed-30" in labels:
                speed_limit = 30
            

            agent.update_information(world, speed_limit)
                    
            if len(agent.get_local_planner().waypoints_queue) < num_min_waypoints and args.loop:
                agent.reroute(spawn_points)
                tot_target_reached += 1
                print("No more waypoints, ending simulation")

            elif len(agent.get_local_planner().waypoints_queue) == 0 and not args.loop:
                print("Target reached, stopping simulation")
                break

            speed_limit = world.player.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)
            control = agent.run_step()
            world.player.apply_control(control)
            # world.player.set_autopilot(True)
            # change_speed(world.player, desired_speed)
    finally:
        if world is not None:
            world.destroy()

        pygame.quit()
 