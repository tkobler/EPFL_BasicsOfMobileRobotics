from platform import node
import time
import math 
import constants as cte

# --- CONFIGURATION CONSTANTS ---

# Thresholds

PROX_MAX = 4500 # crash, approx max val for Thymio IR sensors
OBST_THRESHOLD = 2000  # min val, start considering something an obstacle
DANGER_THRESHOLD = 3500  # collision imminent 
NO_GROUND_THRESHOLD = 50  # min val, start considering no ground below
PROX_MAX = 4500         
OBST_THRESHOLD = 2000   
DANGER_THRESHOLD = 3500 

# Potential field values
REP_GAIN = 1.2          
ATT_GAIN = 1.0          
TANGENTIAL_GAIN = 8.0  
COLLISION_DISTANCE = 1200 

# Memory settings (Fixes the blind spot issue)
OBSTACLE_MEMORY_DELAY = 1.0  # Seconds to keep avoiding after sensor clear
ESCAPE_DURATION = 1.0        # MODIFIED: Time to drive straight after clearing the obstacle (increased to 4.0s)

# Follow or avoid 
TARGET_REACHED_DIST_MM = 30.0 
PATH_TURN_GAIN = 200.0 
AVOID_TURN_GAIN = 150.0 
AVOID_BASE_SPEED = cte.MAIN_MOTOR_SPEED * 0.4

# approx angles (rad) of the 5 front IR sensors in robot frame
SENSOR_ANGLES = [
    -40.0 * math.pi / 180.0,
    -20.0 * math.pi / 180.0,
     0.0,
     20.0 * math.pi / 180.0,
     40.0 * math.pi / 180.0,
]

# Lateral Clearance Threshold 
LATERAL_CLEARANCE_THRESHOLD = 800 


# --- MOTION CONTROL CLASS ---

class MotionControl:
    def __init__(self, node=None, client=None):
        self.node = node
        self.client = client
        self.path = []
        self.current_target_idx = 1

        self.speed_left = 0
        self.speed_right = 0
        self.speed_left_mes = 0
        self.speed_right_mes = 0
        self.time = time.time()

        # Memory variables (Obstacle avoidance state)
        self.last_obst_time = 0.0          
        self.last_obst_side = 0            
        self.avoiding_state = False      
        self.turn_speed = AVOID_TURN_GAIN  
        
        # Escape State variables
        self.is_escaping = False        
        self.escape_start_time = 0.0    


    async def is_kidnapped(self):

        await self.node.wait_for_variables({"prox.ground.delta"})
        
        ground = self.node["prox.ground.delta"]
        left, right = ground[0], ground[1]

        # Threshold: if both values are near zero → no ground detected
        # Kidnapped if both sensors see nothing
        return left < NO_GROUND_THRESHOLD and right < NO_GROUND_THRESHOLD
    
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
        print(f"Current target: {target_x}, {target_y}")

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

        # NEW LOGIC : TURN UNTIL OBSTACLES ARE NOT VISIBLE ANYMORE (THEN ESCAPE)

        if is_physically_visible:
            self.last_obst_time = time.time()
            self.avoiding_state = True
            self.is_escaping = False # If we see it, we are not escaping!
            return True
        
        time_since_detection = time.time() - self.last_obst_time

        if self.avoiding_state and time_since_detection < ESCAPE_DURATION:
            return True
        else:
            # Time is up, we are safe to go back to global nav
            self.avoiding_state = False
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

        # CASE A: OBSTACLE VISIBLE -> TURN AWAY
        if physically_detected:
            # Reset escape timer (we are still dealing with the object)
            self.last_obst_time = time.time()
            
            # Simple decision: Left or Right?
            # Sum sensors on left vs right to decide direction
            # Left side: indices 0, 1. Right side: indices 3, 4. Center: 2
            left_sum = front_vals[0] + front_vals[1]
            right_sum = front_vals[3] + front_vals[4]

            if left_sum > right_sum:
                # Obstacle is on the LEFT -> Turn RIGHT (Push -1)
                self.last_obst_side = 1.0 
            else:
                # Obstacle is on the RIGHT -> Turn LEFT (Push 1)
                self.last_obst_side = -1.0
            
            # Apply turn based on side
            base = cte.MAIN_MOTOR_SPEED
            v_left = base + (self.turn_speed * self.last_obst_side)
            v_right = base - (self.turn_speed * self.last_obst_side)

            await self.node.set_variables(motors(v_left, v_right))
            return
        
        # CASE B: OBSTACLE NOT VISIBLE -> ESCAPE
        else:
            # We are in the escape duration (guaranteed by see_local_obstacle returning True)
            
            # Drive Straight Forward
            # We can add a tiny bias away from the last known obstacle side if desired,
            # but you requested straight.
            
            v_left = cte.MAIN_MOTOR_SPEED
            v_right = cte.MAIN_MOTOR_SPEED
            
            await self.node.set_variables(motors(v_left, v_right))
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
        self.current_target_idx = 1
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