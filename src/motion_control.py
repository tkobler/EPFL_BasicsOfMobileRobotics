#import necessary libraries
#import numpy as np

kidnap_treshold = 5  # Example threshold value

class MotionControl:
    def __init__(self):
        pass

    def is_kiddnapped(self, actual_pos, prev_pos):
        # Check if there is a kidnapping event
        return
    
    def folow_path(self, path):
        # Follow a given path
        return

    #getters and setters can be added as needed
    def get_displacement(self):
        # Return current wheel speeds
        return  
    
    def stop(self):
        # Stop the robot
        return
    
    def is_goal_reached(self, current_pos, goal_pos):
        # Check if the goal is reached
        return


# test code
# you can run python motion_control.py to test a specific function without compiling the entire project
if __name__ == "__main__": 
    mc = MotionControl()
    print("Is robot kidnapped?", mc.is_robot_kiddnapped())
    print("Current displacement:", mc.get_displacement())