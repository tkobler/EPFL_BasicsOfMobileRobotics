from platform import node
import time
import math 
import constants as cte

# --- CONFIGURATION CONSTANTS ---

# Thresholds
PROX_MAX = 4500         
OBST_THRESHOLD = 2000   
DANGER_THRESHOLD = 3500 

# Potential field values
REP_GAIN = 1.2          
ATT_GAIN = 1.0          
TANGENTIAL_GAIN = 10.0  
COLLISION_DISTANCE = 1500 

# Memory settings (Fixes the blind spot issue)
OBSTACLE_MEMORY_DELAY = 1.0  # Seconds to keep avoiding after sensor clear
ESCAPE_DURATION = 1.0        # NEW: Time to drive straight after clearing the obstacle

# Follow or avoid 
TARGET_REACHED_DIST_MM = 30.0 
PATH_TURN_GAIN = 200.0 
AVOID_TURN_GAIN = 250.0 
AVOID_BASE_SPEED = cte.MAIN_MOTOR_SPEED * 0.7 

# approx angles (rad) of the 5 front IR sensors in robot frame
SENSOR_ANGLES = [
    -40.0 * math.pi / 180.0,
    -20.0 * math.pi / 180.0,
     0.0,
     20.0 * math.pi / 180.0,
     40.0 * math.pi / 180.0,
]

# --- MOTION CONTROL CLASS ---

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

        # Memory variables (Obstacle avoidance state)
        self.last_obst_time = 0.0          
        self.last_obst_side = 0            
        self.avoiding_state = False        
        
        # NEW: Escape State variables
        self.is_escaping = False        # Flag: Are we in the 2-second straight escape phase?
        self.escape_start_time = 0.0    # Timestamp when the escape started


    def is_kidnapped(self, actual_pos, prev_pos):
        x, y, _ = actual_pos
        x_prev, y_prev, _ = prev_pos
        dist = math.hypot(x - x_prev, y - y_prev)
        return dist > cte.KIDNAP_THRESHOLD
    
    # ----------------------------------------------------------------------
    ### Global navigation ================
    # ----------------------------------------------------------------------
    async def follow_path(self, current_pos):
        """
        Follow the global path. 
        """
        # Reset avoidance states when safely back on path
        self.avoiding_state = False 
        self.is_escaping = False

        if not self.path or self.current_target_idx >= len(self.path):
            await self.stop()
            return
        
        x, y, theta = current_pos
        target_x, target_y = self.path[self.current_target_idx]

        dx = target_x-x
        dy = target_y-y
        dist = math.hypot(dx, dy)

        if dist < TARGET_REACHED_DIST_MM:
             self.current_target_idx +=1 
             if self.current_target_idx >= len(self.path): 
                  await self.stop()
                  return
             target_x, target_y = self.path[self.current_target_idx]
             dx = target_x-x
             dy = target_y-y

        ang_goal = math.atan2(dy, dx)
        ang_error = ang_goal-theta
        ang_error = math.atan2(math.sin(ang_error), math.cos(ang_error))

        base = cte.MAIN_MOTOR_SPEED
        turn = PATH_TURN_GAIN * ang_error

        v_left = base - turn
        v_right = base + turn

        self.left_command = v_left
        self.right_command = v_right

        await self.node.set_variables(motors(v_left, v_right))
        return

    # ----------------------------------------------------------------------
    ### Local obstacle avoidance ========
    # ----------------------------------------------------------------------
    async def see_local_obstacle(self):
        """
        Returns True if a local obstacle is detected OR if we are in the 
        memory retention period or escape phase.
        """
        await self.node.wait_for_variables({"prox.horizontal"})
        prox = self.node["prox.horizontal"] 
        front_vals = prox[0:5] 
        
        is_physically_visible = max(front_vals) > OBST_THRESHOLD
        
        # If physically visible, update memory time and state
        if is_physically_visible:
            self.last_obst_time = time.time()
            self.avoiding_state = True
            self.is_escaping = False # If we see it, we are not escaping!
            return True
        
        # If sensors are clear, check if we should still be in avoidance mode
        if self.avoiding_state or self.is_escaping:
            if (time.time() - self.last_obst_time) < OBSTACLE_MEMORY_DELAY or self.is_escaping:
                return True
            else:
                self.avoiding_state = False # Memory expired, safe to return False
                self.is_escaping = False
                return False
                
        return False

    async def avoid_local_obstacle(self, current_pos):
        ''' 
        Uses a Tangential Potential Field and the Straight-Line Escape State.
        '''
        
        # STEP 1: Read sensors
        await self.node.wait_for_variables({"prox.horizontal"})
        prox = self.node["prox.horizontal"]
        front_vals = prox[0:5]
        
        physically_detected = max(front_vals) > OBST_THRESHOLD
        Fx_rep, Fy_rep = 0.0, 0.0

        # --- ESCAPE STATE EXECUTION (HIGHEST PRIORITY) ---
        if self.is_escaping:
            # 1. Check if duration is over or obstacle is suddenly visible
            if (time.time() - self.escape_start_time) > ESCAPE_DURATION or physically_detected:
                self.is_escaping = False
                if physically_detected:
                     # If it reappeared, continue to full avoidance below
                     pass 
                else:
                     # Escape finished successfully, stop avoidance 
                     self.avoiding_state = False 
                     return # End the avoidance cycle immediately, let main switch to follow_path
            
            # 2. Execute straight movement with slight lateral push
            if self.is_escaping:
                # Strong forward force and a slight correction away from the obstacle side
                Fx = 2.0 
                Fy = 0.5 * self.last_obst_side 
                
                # Apply movement commands
                force_angle = math.atan2(Fy, Fx) 
                ang_err = math.atan2(math.sin(force_angle), math.cos(force_angle))
                
                base = cte.MAIN_MOTOR_SPEED 
                turn = PATH_TURN_GAIN * ang_err

                v_left = base - turn
                v_right = base + turn

                await self.node.set_variables(motors(v_left, v_right))
                await self.client.sleep(0.05)
                return # Skip PF calculation for the next 2 seconds
        # --- END ESCAPE STATE EXECUTION ---

        # --- POTENTIAL FIELD CALCULATION ---
        
        if physically_detected:
            
            # === STEP A.1: Calculate RAW Repulsive Force (Tangential Force Generation) ===
            max_val = 0
            max_idx = -1
            
            for i, val in enumerate(front_vals):
                if val > max_val:
                    max_val = val
                    max_idx = i
            
            if max_idx != -1 and max_val > COLLISION_DISTANCE:
                
                ang_obs = SENSOR_ANGLES[max_idx]
                magnitude = TANGENTIAL_GAIN * (max_val - COLLISION_DISTANCE) / PROX_MAX
                
                # Decision for turn direction
                if max_idx in [0, 1]:  # Obstacle on the left side
                    push_direction = -1.0 # Push RIGHT (Turn Left)
                else:                  # Obstacle on the right side (or center)
                    push_direction = 1.0  # Push LEFT (Turn Right)
                    
                tangential_angle = ang_obs + push_direction * (math.pi / 2)
                
                Fx_rep = magnitude * math.cos(tangential_angle)
                Fy_rep = magnitude * math.sin(tangential_angle)
                
                # Update Memory
                self.last_obst_side = push_direction
                self.last_obst_time = time.time()
            
            # Emergency Stop logic
            if max(front_vals) > DANGER_THRESHOLD:
                v_left = -cte.MAIN_MOTOR_SPEED * self.last_obst_side * 0.5
                v_right = cte.MAIN_MOTOR_SPEED * self.last_obst_side * 0.5
                await self.node.set_variables(motors(v_left, v_right))
                await self.client.sleep(0.05)
                return

        else:
            # === CASE B: Blind Spot (Memory Mode) - TRANSITION TO ESCAPE ===
            # Sensors are clear, but avoidance memory is still active.
            if self.avoiding_state:
                # The turn is complete. Trigger the straight escape phase NOW.
                self.is_escaping = True
                self.escape_start_time = time.time()
                
                # Since the main logic checks the ESCAPE STATE first, this
                # function will exit and re-run immediately, executing the 
                # escape logic block above.
                return 

            # If memory just expired (self.avoiding_state is False here), Fx_rep, Fy_rep remain 0.0

        # STEP 3: Attractive Force (Goal)
        # We only reach here if the robot is *not* escaping, either in full avoidance or memory fade-out
        if physically_detected:
            current_att_gain = ATT_GAIN * 0.2
        else:
            current_att_gain = ATT_GAIN * 0.8 

        x, y, theta = current_pos
        target_x, target_y = self.path[self.current_target_idx]
        dx = target_x - x
        dy = target_y - y

        ang_goal = math.atan2(dy, dx)
        ang_goal_robot = ang_goal - theta
        ang_goal_robot = math.atan2(math.sin(ang_goal_robot), math.cos(ang_goal_robot))

        Fx_att = current_att_gain * math.cos(ang_goal_robot)
        Fy_att = current_att_gain * math.sin(ang_goal_robot)
        
        # Resultant Force
        Fx = Fx_att + Fx_rep
        Fy = Fy_att + Fy_rep

        if Fx == 0.0 and Fy == 0.0:
            await self.move() 
            return
         
        # Translate to wheel speeds
        force_angle = math.atan2(Fy, Fx) 
        
        # Tweak: Dampen angle change when repulsive forces are fading
        if not physically_detected:
            force_angle = force_angle * 0.8 

        ang_err = math.atan2(math.sin(force_angle), math.cos(force_angle))

        base = AVOID_BASE_SPEED * max(0.2, math.cos(ang_err)) 
        turn = AVOID_TURN_GAIN * ang_err

        v_left = base - turn
        v_right = base + turn

        self.left_command = v_left
        self.right_command = v_right

        await self.node.set_variables(motors(v_left, v_right))
        await self.client.sleep(0.05)
        return 

    # ----------------------------------------------------------------------
    ### Getters and Setters ================
    # ----------------------------------------------------------------------
    async def get_speed(self):
        await self.node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
        self.speed_left_mes = self.node["motor.left.speed"] * cte.SPEED_TO_MMS 
        self.speed_right_mes = self.node["motor.right.speed"] * cte.SPEED_TO_MMS
        await self.client.sleep(0.01)
        return self.speed_left_mes, self.speed_right_mes
    
    def get_time(self):
        current_time = time.time()
        delta_t = current_time - self.time
        self.time = current_time
        return delta_t

    def set_path(self, path):
        self.path = path
        self.current_target_idx = 0
        self.avoiding_state = False 
        self.is_escaping = False
        return

    async def stop(self):
        await self.node.set_variables(motors(0, 0))
        return

    async def move(self):
        await self.node.set_variables(motors(cte.MAIN_MOTOR_SPEED, cte.MAIN_MOTOR_SPEED))
        return

# --- HELPER FUNCTION ---
def motors(left, right):
    return {
        "motor.left.target": [int(left * cte.CALIB_LEFT_WHEEL)],
        "motor.right.target": [int(right * cte.CALIB_RIGHT_WHEEL)],
    }