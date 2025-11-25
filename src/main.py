from tdmclient import ClientAsync, aw # Warning: for the report, imports must be clearly stated at the top of the file
from filter import Filter
from global_nav import GlobalNav
from motion_control import MotionControl
from vision import Map
import constants as cte


async def main():
    try: 
        client = ClientAsync()
        node = await client.wait_for_node()
        await node.lock()

        # ======================================
        # Initialize modules
        # ======================================
        map = Map()
        map.init_map()
        thymio, goal, obstacles = [],[],[] # positions of thymios, goals and obstacles
        thymio = map.get_thymio_position() # pos x,y and angle in radians
        goal = map.get_goal_position() # pos x,y 
        obstacles = map.get_obstacles_positions() # list of obstacles ((x,y), (x,y))

        filter = Filter(thymio)
    
        global_nav = GlobalNav()
        path = global_nav.find_optimal_path(thymio, goal, obstacles) # list of pos x,y en mm

        motion_control = MotionControl(node, client)
        motion_control.set_path(path)

        goal_not_reached = True

        # ======================================
        # Main loop
        # ======================================
        while goal_not_reached:
            # sensing
            pos_measure = map.get_thymio_position() # pos x,y and angle in radians
            left_speed_mes, right_speed_mes = await motion_control.get_speed() # wheel speeds in mm/s
            delta_t = motion_control.get_time() # time elapsed since last measure in s

            # pose estimation
            pos_estim = filter.update(pos_measure, delta_t, left_speed_mes, right_speed_mes)

            # is goal reached ?
            if global_nav.is_goal_reached(pos_estim, goal):
                goal_not_reached = False
                await motion_control.stop()
                break

            # is there a kidnapping?
            if motion_control.is_kidnapped(pos_estim, thymio):
                 path = global_nav.find_optimal_path(pos_estim, goal, obstacles)
                 motion_control.set_path(path)
            
            thymio = pos_estim
            # movment 
            if (await motion_control.see_local_obstacle()): 
                # Avoid local obstacle
                await motion_control.avoid_local_obstacle(thymio)
            else: 
                # Follow the global path
                await motion_control.follow_path(thymio)

    except Exception as e:
        print(f"An error occurred: {e}")

    # Exit program
    finally:
        # Stop the robot    
        await node.stop()
        # Unlock the robot
        await node.unlock()

if __name__ == "__main__": # runs the main function if this file is executed directly
    with ClientAsync() as client:
        client.run_async_program(main)