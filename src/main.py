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
        carte = Map()
        carte.init_map(camera_id=0)
        thymio, goal, obstacles = [],[],[] # positions of thymios, goals and obstacles
        print("Viser la map avec la caméra, les 4 markers dans les coins doivent être visibles.")
        print("Appuyez sur 's' pour initialiser la map")
        print("Puis quand la map apparait à l'écran, appuyer à nouveau sur 's' pour valider")
        while True:
            key = carte.afficher_video()
            if key == ord('s'):
                break
        obstacles = carte.get_obstacles_positions() # list of obstacles ((x,y), (x,y))
        print("Map initialisée avec succès.")
        print("Placez le Thymio sur la map à la position de départ.")
        print("Appuyez sur 'p' pour initialiser la position du Thymio et commencer la navigation")
        while True:
            key = carte.afficher_video()
            if key == ord('p'):
                break
        search_pos = True
        while search_pos:
            thymio = carte.get_thymio_position() # pos x,y and angle in radians
            if thymio != []:
                search_pos = False
        goal = carte.get_goal_position() # pos x,y 

        filter = Filter(thymio)
    
        global_nav = GlobalNav()
        path = global_nav.find_optimal_path(thymio, goal, obstacles) # list of pos x,y en mm
        global_nav.plot_map(obstacles, path, [thymio[0], thymio[1]], goal)
        print("Chemin calculé avec succès.")
        print(path)

        motion_control = MotionControl(node, client)
        motion_control.set_path(path)

        goal_not_reached = True

        # ======================================
        # Main loop
        # ======================================
        while goal_not_reached:
            # sensing
            pos_measure = carte.get_thymio_position() # pos x,y and angle in radians
            left_speed_mes, right_speed_mes = await motion_control.get_speed() # wheel speeds in mm/s
            delta_t = motion_control.get_time() # time elapsed since last measure in s
            #print(f"Measured pos: {pos_measure}, Left speed: {left_speed_mes}, Right speed: {right_speed_mes}, Delta t: {delta_t}")

            # position estimation
            pos_estim = filter.update(pos_measure, delta_t, left_speed_mes, right_speed_mes)
            #print(f"Estimated pos: {pos_estim}")

            # position display
            carte.dessin_point(pos_estim[0],pos_estim[1], pos_estim[2])
            carte.dessin_chemin(path)
            carte.afficher_video()

            # is goal reached ?
            if global_nav.is_goal_reached(pos_estim, goal):
                goal_not_reached = False
                print("Goal reached!")
                await motion_control.stop()
                break

            # is there a kidnapping?
            if motion_control.is_kidnapped(pos_estim, thymio):
                print("Kidnapping detected! Stopping the robot.")
                await motion_control.stop()
                print("Please place the robot back on the map and press 'r' to resume.")
                while True:
                    key = carte.afficher_video()
                    if key == ord('r'):
                        break
                # Reinitialize position
                search_pos = True
                while search_pos:
                    pos_estim = carte.get_thymio_position() # pos x,y and angle in radians
                    if pos_estim != []:
                        search_pos = False
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