#import necessary libraries
#import numpy as np

class Map:
    def __init__(self):
        pass

    def init_map(self):
        # Function that use the camera to scan the environment and create a map with the position of global obstacles, 
        # start point, end point 
        return
    

    #getters and setters can be added as needed
    def get_thymio_position(self):
        # Function that returns the current position of the Thymio robot on the map
        return
    
    def get_goal_position(self):
        # Function that returns the positions of the goals on the map
        return
    
    def get_obstacles_positions(self):
        # Function that returns the positions of the obstacles on the map
        return

    def goal_not_reached(self, thymio_position, goal_position):
        # Function that checks if the Thymio robot has reached the goal
        return True



# test code
# you can run python vision.py to test a specific function without compiling the entire project
if __name__ == "__main__": 
    map_instance = Map()
    map_instance.init_map()
    print("Map initialized.")