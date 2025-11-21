#import necessary libraries
#import numpy as np


from platform import node
import time
import constants as cte


kidnap_treshold = 5  # Example threshold value
CALIB_SPEED_CMD = 200  


class MotionControl:
    def __init__(self, node=None, client=None):
        self.node = node
        self.client = client
        self.path = []
        self.speed_left = 0
        self.speed_right = 0
        self.speed_left_mes = 0
        self.speed_right_mes = 0
        self.time = time.time()
        self.left_command = 0
        self.right_command = 0


    def is_kidnapped(self, actual_pos, prev_pos):
        # Check if there is a kidnapping event
        return
    
    ### Global navigation ================
    async def follow_path(self, current_pos):
        # Follow a given path
        # set speed des deux moteurs pour atteindre l'objectif du prochain élément dans le self.path
        return

    ### Local obstacle avoidance ========
    async def see_local_obstacle(self):
        # Logic to detect local obstacles
        return

    async def avoid_local_obstacle(self, current_pos):
        # Logic to avoid the local obstacle
        # On fait une manoeuvre d'évitement 
        return 

    ### getters and setters ================
    async def get_speed(self):
        # Return current wheel speeds
        await self.node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
        self.speed_left_mes = self.node["motor.left.speed"] * cte.SPEED_TO_MMS  # speed in mm/s
        self.speed_right_mes = self.node["motor.right.speed"] * cte.SPEED_TO_MMS  # speed in mm/s
        await self.client.sleep(0.01)
        return self.speed_left_mes, self.speed_right_mes
    
    def get_time(self):
        # Return time elapsed since last call
        current_time = time.time()
        delta_t = current_time - self.time
        self.time = current_time
        return delta_t
    
    def get_commanded_speed(self):
        # Return last commanded wheel speeds
        return self.left_command, self.right_command

    def set_path(self, path):
        # Set a new path for the robot
        self.path = path
        return
    

    ### Navigation in general ================
    async def stop(self):
        # Stop the robot
        await self.node.set_variables(motors(0, 0))
        return

    async def move(self):
        # Logic to move the robot based on current path and obstacle detection
        # Ensure we see motor speeds
        await self.node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
        self.speed_left_mes = self.node["motor.left.speed"] # speed in Thymio units
        self.speed_right_mes = self.node["motor.right.speed"] # speed in Thymio units
        # print("Moving forward")
        await self.node.set_variables(motors(CALIB_SPEED_CMD, CALIB_SPEED_CMD))
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