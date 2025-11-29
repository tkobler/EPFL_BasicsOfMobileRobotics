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

OBST_THRESHOLD = 1500   # min val, start considering something an obstacle
DANGER_THRESHOLD = 3500 # collision imminent 

# Follow or avoid 

TARGET_REACHED_DIST_MM = 30.0  # Target is reached radius (stay on trajectory), switch to next
PATH_TURN_GAIN = 200.0         # How much to turn to align with target 
AVOID_BASE_SPEED = cte.MAIN_MOTOR_SPEED * 0.7  # a bit slower when avoiding collision (not strictly required)

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
        self.current_target_idx = 0

        self.speed_left = 0
        self.speed_right = 0
        self.speed_left_mes = 0
        self.speed_right_mes = 0
        self.time = time.time()

        self.avoiding = False          # True while doing the big loop
        self.avoid_side = "LEFT"       # "LEFT" or "RIGHT"
        self.avoid_start_time = 0.0    # when we started the big loop

        self.left_command = 0
        self.right_command = 0



    def is_kidnapped(self, actual_pos, prev_pos):
        # Check if there is a kidnapping event
        x, y, _ = actual_pos
        x_prev, y_prev, _ = prev_pos

        dx = x - x_prev
        dy = y - y_prev

        dist = math.hypot(dx, dy)
        
        return dist > cte.KIDNAP_THRESHOLD
    
    ### Global navigation ================
    async def follow_path(self, current_pos):
        """
        Follow the global path. It is composed of multiple targets. Set motor speeds 
        to go towards the next target in self.path.
        current_pos: (x, y, theta) in environment frame (mm, rad)
        """
        if self.avoiding:
            return

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

        ### Local obstacle avoidance: ONE BIG LOOP, time-based, dynamic side ###
    async def avoid_local_obstacle(self, current_pos):
        """
        On obstacle detection:
          - Pick LEFT or RIGHT based on where the obstacle is stronger.
          - For BIG_LOOP_TIME seconds, drive a wide arc in that direction.
          - Then stop avoiding and give control back to follow_path.

        No angles, no multi-phase state, just one big loop.
        """

        # --- PARAMETERS TO TUNE THE SIZE OF THE LOOP ---
        BIG_LOOP_TIME = 4.0          # seconds spent driving the big arc (try 3.0–4.0)
        CURVE_RATIO   = 0.9         # 0.85 vs 1.0 => wide arc (NOT tiny circle)
        base_speed    = cte.MAIN_MOTOR_SPEED

        # --- READ FRONT SENSORS ---
        await self.node.wait_for_variables({"prox.horizontal"})
        prox = self.node["prox.horizontal"]      # 7 values
        front_vals = prox[0:5]                   # front 5
        max_front = max(front_vals)

        now = time.time()

        print(f"[AVOID] avoiding={self.avoiding}, max_front={max_front}") # DEBUGGINGH 

        # -------------------------------------------------------
        # 1) If we're NOT currently avoiding, decide whether to START
        # -------------------------------------------------------
        if not self.avoiding:
            if max_front > OBST_THRESHOLD:
                # Choose side dynamically based on which side sees more
                # Assumes front_vals[0..4] go from LEFT to RIGHT.
                left_sum  = front_vals[0] + front_vals[1]
                right_sum = front_vals[3] + front_vals[4]

                if left_sum > right_sum:
                    # obstacle more on the LEFT -> go around on the RIGHT
                    self.avoid_side = "RIGHT"
                else:
                    # obstacle more on the RIGHT (or equal) -> go around on the LEFT
                    self.avoid_side = "LEFT"

                self.avoiding = True
                self.avoid_start_time = now

            else:
                # Not avoiding, nothing to do
                return

        # -------------------------------------------------------
        # From here on, we ARE in avoiding = True
        # -------------------------------------------------------

        # Emergency: if something is insanely close, spin in place
        if max_front > DANGER_THRESHOLD:
            v_left = -base_speed
            v_right = base_speed
            self.left_command = v_left
            self.right_command = v_right
            await self.node.set_variables(motors(v_left, v_right))
            await self.client.sleep(0.05)
            return

        # Check if we've done the full big loop
        elapsed = now - self.avoid_start_time
        if elapsed >= BIG_LOOP_TIME:
            # Done with the big loop, stop avoiding
            self.avoiding = False
            return

        # -------------------------------------------------------
        # 2) DRIVE ONE BIG WIDE ARC AROUND THE OBSTACLE
        # -------------------------------------------------------
        if self.avoid_side == "LEFT":
            # Wide LEFT arc: left slightly slower than right
            v_left  = CURVE_RATIO * base_speed
            v_right = 1.0 * base_speed
        else:  # "RIGHT"
            # Wide RIGHT arc: right slightly slower than left
            v_left  = 1.0 * base_speed
            v_right = CURVE_RATIO * base_speed

        self.left_command = v_left
        self.right_command = v_right

        ################DEBUG################

        elapsed = now - self.avoid_start_time
        if elapsed >= BIG_LOOP_TIME:
            print(f"[AVOID] finished big loop after {elapsed:.2f}s")
            self.avoiding = False
            return
        
        ################DEBUG################


        await self.node.set_variables(motors(v_left, v_right))
        await self.client.sleep(0.05)


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
