#import necessary libraries
#import numpy as np


from platform import node
import constants as cte


kidnap_treshold = 5  # Example threshold value
CALIB_SPEED_CMD = 200  


class MotionControl:
    def __init__(self):
        self.path = []
        self.speed_left = 1
        self.speed_right = 1
        self.speed_left_mes = 1
        self.speed_right_mes = 1


    def is_kidnapped(self, actual_pos, prev_pos):
        # Check if there is a kidnapping event
        return
    
    ### Global navigation ================
    async def follow_path(self, current_pos, node):
        # Follow a given path
        # set speed des deux moteurs pour atteindre l'objectif du prochain élément dans le self.path
        return

    ### Local obstacle avoidance ========
    async def see_local_obstacle(self, node):
        # Logic to detect local obstacles
        return

    async def avoid_local_obstacle(self, current_pos, node):
        # Logic to avoid the local obstacle
        # On fait une manoeuvre d'évitement 
        return 
    
    async def stop(self, node):
        # Stop the robot
        return

    ### getters and setters ================
    def get_speed(self):
        # Return current wheel speeds
        return self.speed_left_mes, self.speed_right_mes

    def set_path(self, path):
        # Set a new path for the robot
        self.path = path
        return
    

    ### Navigation in general ================
    async def stop(self, node):
        # Stop the robot
        return

    async def move(self, node):
        # Logic to move the robot based on current path and obstacle detection
        # Ensure we see motor speeds
        await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
        self.speed_left_mes = node["motor.left.speed"] # speed in Thymio units
        self.speed_right_mes = node["motor.right.speed"] # speed in Thymio units
        # print("Moving forward")
        # print(f"Left motor speed: {self.speed_left_mes}, Right motor speed: {self.speed_right_mes}")
        await node.set_variables(motors(CALIB_SPEED_CMD, CALIB_SPEED_CMD))
        return


### Helper function to use the motors
def motors(left, right):
    return {
        "motor.left.target": [int(left * cte.CALIB_LEFT_WHEEL)],
        "motor.right.target": [int(right * cte.CALIB_RIGHT_WHEEL)],
    }


# test code
# you can run python motion_control.py to test a specific function without compiling the entire project
if __name__ == "__main__": 
    mc = MotionControl()
    print("Is robot kidnapped?", mc.is_robot_kiddnapped())
    print("Current displacement:", mc.get_displacement())