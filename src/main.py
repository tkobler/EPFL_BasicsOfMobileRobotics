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
        thymio = map.get_thymio_position() # pos x,y
        goal = map.get_goal_position() # pos x,y 
        obstacles = map.get_obstacles_positions() # list of pos x,y
        path = global_nav.find_optimal_path(thymio, goal, obstacles) # list of pos x,y 
        motion_control.set_path(path)

        # Main loop
        while goal_not_reached:
            # sensing
            pos_measure = map.get_thymio_position()
            left_speed, right_speed = motion_control.get_speed()

            # pose estimation
            pos_estim = filter.update(pos_measure, left_speed, right_speed)

            # is goal reached ?
            if global_nav.is_goal_reached(thymio, goal):
                goal_not_reached = False
                motion_control.stop()
                break

            # is there a kidnapping?
            if motion_control.is_kidnapped(pos_measure, thymio):
                 pos_estim = pos_measure
                 path = global_nav.find_optimal_path(pos_estim, goal, obstacles)
                 motion_control.set_path(path)
            
            thymio = pos_estim
            # movment 
            if motion_control.see_local_obstacle(): 
                # Avoid local obstacle
                motion_control.avoid_local_obstacle()
            else: 
                # Follow the global path
                motion_control.follow_path()


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