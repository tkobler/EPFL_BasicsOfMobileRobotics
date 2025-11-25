#import necessary libraries
#import numpy as np


from platform import node
import time
import math 
import constants as cte

'''
This part has the values that the sensors take in order to detect wether or not it needs to avoid.
'''

# Thresholds

PROX_MAX = 4500 # crash, approx max val for Thymio IR sensors
OBST_THRESHOLD = 2000  # min val, start considering something an obstacle
DANGER_THRESHOLD = 3500  # collision imminent 

# Potential field values

REP_GAIN = 1.2 # Strength of repulsive forces
ATT_GAIN = 1.0 # Strength of attractive force during avoidance

# Follow or avoid 

TARGET_REACHED_DIST_MM = 30.0 # Target is reached radius (stay on trajectory), switch to next
PATH_TURN_GAIN = 200.0 # How much to turn to align with target 
AVOID_TURN_GAIN = 250.0 # How much to turn to avoid collision
AVOID_BASE_SPEED = cte.MAIN_MOTOR_SPEED * 0.7  # a bit slower when avoiding collision

# approx angles (rad) of the 5 front IR sensors in robot frame, no need for those in th e back
SENSOR_ANGLES = [
    -40.0 * math.pi / 180.0,
    -20.0 * math.pi / 180.0,
     0.0,
     20.0 * math.pi / 180.0,
     40.0 * math.pi / 180.0,
]

class MotionControl:
    def __init__(self, node=None, client=None):
        self.node = node
        self.client = client
        self.path = []

        # since the path is a list [(x0, y0), (x1, y1), (x2, y2), ...] we are aiming at the first target on the path 
        self.current_target_idx = 0

        self.speed_left = 0
        self.speed_right = 0
        self.speed_left_mes = 0
        self.speed_right_mes = 0
        self.time = time.time()


    def is_kidnapped(self, actual_pos, prev_pos):
        # Check if there is a kidnapping event
        a = cte.KIDNAP_THRESHOLD
        return False 
    
    ### Global navigation ================
    async def follow_path(self, current_pos):
        """
        Follow the global path. It is composed of multiple targets. Set motor speeds 
        to go towards the next target in self.path.
        current_pos: (x, y, theta) in environment frame (mm, rad)
        """

        # STEP 1 : CHECK WHERE YOU ARE AND WHERE YOU NEED TO GO

        if not self.path or self.current_target_idx >= len(self.path):
                    # If ther is no path the robot has to stop
                    await self.stop()
                    return
        
        x, y, theta = current_pos
        target_x, target_y = self.path[self.current_target_idx]

        # Vector to the target
        dx = target_x-x
        dy = target_y-y
        dist = math.hypot(dx, dy)

        # STEP 2 : CHECK IF YOU REACHED THE DESTINATION, IF YES COMPUTE NEXT

        # If the target is reached, scope for next target 
        if dist < TARGET_REACHED_DIST_MM:
             self.current_target_idx +=1 # switch to next
             if self.current_target_idx >= len(self.path): # was it the last one 
                  await self.stop()
                  return
             # get vector again
             target_x, target_y = self.path[self.current_target_idx]
             dx = target_x-x
             dy = target_y-y
             dist = math.hypot(dx, dy)

        # STEP 3 : ACTUALLY MOVE

        # Get the right angle
        ang_goal = math.atan2(dy, dx)
        ang_error = ang_goal-theta
        ang_error = math.atan2(math.sin(ang_error), math.cos(ang_error))
        # atan2 returns the angle theta between the positive x-axis and the ray from the origin to the point (x, y), confined to (−π, π]

        # Now adjust the speeds
        base = cte.MAIN_MOTOR_SPEED

        turn = PATH_TURN_GAIN * ang_error

        v_left = base - turn
        v_right = base + turn

        self.left_command = v_left
        self.right_command = v_right

        await self.node.set_variables(motors(v_left, v_right))

        return

    ### Local obstacle avoidance ========
    async def see_local_obstacle(self):
        """
        Returns True if a local obstacle is detected in front of the robot using the
        proximity sensors.
        """
        await self.node.wait_for_variables({"prox.horizontal"})
        prox = self.node["prox.horizontal"]  # List of 7 values of all sensors 
        front_vals = prox[0:5] # We don't need those in the back

        await self.client.sleep(0.01)
        return max(front_vals) > OBST_THRESHOLD

    async def avoid_local_obstacle(self, current_pos):
        ''' 
        Use potential field mapping. 
        - Obstacles are repulsive force vectors.
        - Path is an attractive force vector.
        '''

        # Purely for the sake of testing 
        if not self.path or self.current_target_idx >= len(self.path):
            goal_attraction = False
        else:
            goal_attraction = True

        # STEP 1: Read the data from the proximity sensors 
        await self.node.wait_for_variables({"prox.horizontal"})
        prox = self.node["prox.horizontal"]
        front_vals = prox[0:5]

        # STEP 2: Compute the repulsive force from detected obstacles
        Fx_rep, Fy_rep = 0.0, 0.0
        for val, ang in zip(front_vals, SENSOR_ANGLES): 
            # Zip pairs the lists element wise (angle and data of sensor) 
            if val <= OBST_THRESHOLD:
                # Ignore this reading if below the obstacle threshold 
                continue

            # The magnitude should be proportional to how much the threshold is exceeded 
            mag = REP_GAIN * (val - OBST_THRESHOLD) / max(1.0, (PROX_MAX - OBST_THRESHOLD))

            # The direction of the vector should be opposite to the sensor axis
            Fx_rep += -mag * math.cos(ang)
            Fy_rep += -mag * math.sin(ang)

        # Emergency situation : collision is imminent then spin on the spot until facing away 
        if max(front_vals) > DANGER_THRESHOLD:
            v_left = -cte.MAIN_MOTOR_SPEED
            v_right = cte.MAIN_MOTOR_SPEED
            self.left_command = v_left
            self.right_command = v_right
            await self.node.set_variables(motors(v_left, v_right))
            await self.client.sleep(0.05)
            return
        
        # Compute the attractive force towards target to stay on track 
        Fx_att, Fy_att = 0.0, 0.0
        if goal_attraction:
            x, y, theta = current_pos
            target_x, target_y = self.path[self.current_target_idx]

            dx = target_x - x
            dy = target_y - y

            ang_goal = math.atan2(dy, dx)
            ang_goal_robot = ang_goal - theta
            ang_goal_robot = math.atan2(math.sin(ang_goal_robot), math.cos(ang_goal_robot))

            Fx_att = ATT_GAIN * math.cos(ang_goal_robot)
            Fy_att = ATT_GAIN * math.sin(ang_goal_robot)
        
        # Compute the resulting force 
        Fx = Fx_att + Fx_rep
        Fy = Fy_att + Fy_rep

        if Fx == 0.0 and Fy == 0.0:
            # nothing to do
            await self.stop()
            return
         
        # Translate this force into wheel speeds 
        force_angle = math.atan2(Fy, Fx)  # in robot frame
        force_angle += 10 * math.pi / 180.0
        ang_err = math.atan2(math.sin(force_angle), math.cos(force_angle))

        base = AVOID_BASE_SPEED * max(0.0, math.cos(ang_err))  # slow when turning a lot
        turn = AVOID_TURN_GAIN * ang_err

        v_left = base - turn
        v_right = base + turn

        self.left_command = v_left
        self.right_command = v_right

        await self.node.set_variables(motors(v_left, v_right))
        await self.client.sleep(0.05)

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

    def set_path(self, path):
        # Set a new path for the robot
        self.path = path
        self.current_target_idx = 0
        return
    

    ### Navigation in general ================
    async def stop(self):
        # Stop the robot
        await self.node.set_variables(motors(0, 0))
        self.left_command = 0
        self.right_command = 0
        return

    async def move(self):
        # Logic to move the robot based on current path and obstacle detection
        # Ensure we see motor speeds
        await self.node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
        self.speed_left_mes = self.node["motor.left.speed"] # speed in Thymio units
        self.speed_right_mes = self.node["motor.right.speed"] # speed in Thymio units
        # print("Moving forward")
        await self.node.set_variables(motors(cte.MAIN_MOTOR_SPEED, cte.MAIN_MOTOR_SPEED))
        return


### Helper function to use the motors
def motors(left, right):
    return {
        "motor.left.target": [int(left * cte.CALIB_LEFT_WHEEL)],
        "motor.right.target": [int(right * cte.CALIB_RIGHT_WHEEL)],
    }
