#import necessary libraries
#import numpy as np

kidnap_treshold = 5  # Example threshold value

class MotionControl:
    def __init__(self):
        path = []
        speed_left = 0
        speed_right = 0


    def is_kidnapped(self, actual_pos, prev_pos):
        # Check if there is a kidnapping event
        return
    
    def follow_path(self):
        # Follow a given path
        # set speed des deux moteurs pour atteindre l'objectif du prochain élément dans le self.path
        return
    
    def see_local_obstacle(self):
        # Logic to detect local obstacles
        return

    def avoid_local_obstacle(self):
        # Logic to avoid the local obstacle
        # On fait une manoeuvre d'évitement 
        return 
    
    def stop(self):
        # Stop the robot
        return

    #getters and setters can be added as needed
    def get_speed(self):
        # Return current wheel speeds
        return self.speed_left, self.speed_right

    def set_path(self, path):
        # Set a new path for the robot
        self.path = path
        return

# test code
# you can run python motion_control.py to test a specific function without compiling the entire project
if __name__ == "__main__": 
    mc = MotionControl()
    print("Is robot kidnapped?", mc.is_robot_kiddnapped())
    print("Current displacement:", mc.get_displacement())