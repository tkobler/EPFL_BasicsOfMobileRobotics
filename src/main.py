from tdmclient import ClientAsync, aw # Warning: for the report, imports must be clearly stated at the top of the file
from filter import Filter
from global_nav import GlobalNav
from local_nav import LocalNav
from motion_control import MotionControl
from vision import Map

async def main():
    # remark: see doc to know how to use this method: https://github.com/epfl-mobots/tdm-python/blob/main/doc/lowlevel.md
    try: 
        node = await client.wait_for_node()
        await node.lock()

        # Initialize instances of different modules
        map = Map()
        filter = Filter()
        global_nav = GlobalNav()
        local_nav = LocalNav()
        motion_control = MotionControl()
        thymio, goal, obstacles = [],[],[] # positions of thymios, goals and obstacles
        goal_not_reached = True
    
        # Init the map, filter, global and local navigation modules
        map.init_map()
        thymio = map.get_thymio_position()
        goal = map.get_goal_position()
        obstacles = map.get_obstacles_positions()
        path = global_nav.find_optimal_path(thymio, goal, obstacles)

        # Main loop
        while goal_not_reached:
            # sensing
            new_pos = map.get_thymio_position()
            left_displacement, right_speed = motion_control.get_displacement()

            # pose estimation
            pos_estim = filter.update(new_pos, left_displacement, right_speed)

            # is there a kidnapping?
            if motion_control.is_kidnapped(pos_estim, thymio):
                path = global_nav.find_optimal_path(new_pos, goal, obstacles)
                pos_estim = new_pos

            thymio = pos_estim

            # is goal reached ?
            if motion_control.is_goal_reached(thymio, goal):
                goal_not_reached = False
                motion_control.stop()
                break

            # movment 
            if local_nav.see_obstacle():
                local_nav.avoid_obstacle()
            
            else: 
                # Follow the global path
                motion_control.follow_path(path)


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