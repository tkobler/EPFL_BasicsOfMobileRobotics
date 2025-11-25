import asyncio
import math
import time
import numpy as np
from tdmclient import ClientAsync


# ======================================
# Dead reckoning state (odometry)
# ======================================
robot_state = {
    "x": 0.0,      # mm
    "y": 0.0,      # mm
    "theta": 0.0,  # radians
}

# ======================================
# Parameters (copy from constants.py)
# ======================================
SPEED_TO_MMS = 0.32769  # current guess: mm/s per Thymio speed unit
L = 122.7                            # wheelbase in mm (same as in constants.py)

# Calibration motion (straight line)
CALIB_SPEED_CMD = 200      # motor.left.target = motor.right.target
TARGET_DIST_MM = 500.0     # stop when odom distance >= this (mm)
CALIB_MAX_DURATION = 6.0   # safety timeout in seconds
CALIB_LEFT_WHEEL = 1.05
CALIB_RIGHT_WHEEL = 1.0

DT_NOMINAL = 0.05          # nominal sleep between iterations (s)


# ======================================
# Odometry update (same as constants.py)
# ======================================
def update_robot_state(v_l_raw, v_r_raw, dt):
    """
    Update x, y, theta using real units.
    v_l_raw and v_r_raw are Thymio motor speed units.
    dt is the *actual* time step in seconds.
    """

    # Convert raw wheel speeds to mm/s
    v_l = v_l_raw * SPEED_TO_MMS
    v_r = v_r_raw * SPEED_TO_MMS

    # Linear and angular velocity
    v = (v_r + v_l) / 2.0      # mm/s
    omega = 0 #(v_r - v_l) / L    # rad/s

    # Integrate position
    robot_state["x"] += v * math.cos(robot_state["theta"]) * dt
    robot_state["y"] += v * math.sin(robot_state["theta"]) * dt
    robot_state["theta"] += omega * dt

    # Normalize angle to [-pi, pi]
    robot_state["theta"] = (robot_state["theta"] + np.pi) % (2 * np.pi) - np.pi


def motors(left, right):
    return {
        "motor.left.target": [int(left * CALIB_LEFT_WHEEL)],
        "motor.right.target": [int(right * CALIB_RIGHT_WHEEL)],
    }


# ======================================
# Async calibration routine (linear)
# ======================================
async def run_calibration(node, client):
    # Ensure we see motor speeds
    await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})

    # Reset odometry
    robot_state["x"] = 0.0
    robot_state["y"] = 0.0
    robot_state["theta"] = 0.0

    print("=== Thymio linear calibration ===")
    print(f"Current SPEED_TO_MMS guess: {SPEED_TO_MMS:.6f} mm/s per unit")
    print(f"Motion: left={CALIB_SPEED_CMD}, right={CALIB_SPEED_CMD}")
    print(f"Stop when odom distance >= {TARGET_DIST_MM:.1f} mm "
          f"or after {CALIB_MAX_DURATION:.1f} s (timeout).")
    input("Place the robot on the floor, align it carefully on a straight line, "
          "then press ENTER to start the calibration motion...")

    # Start straight motion
    await node.set_variables(motors(CALIB_SPEED_CMD, CALIB_SPEED_CMD))

    t_start = time.time()
    last_t = t_start

    while True:
        now = time.time()
        dt = now - last_t
        last_t = now

        # Read wheel speeds
        v_l = node["motor.left.speed"]
        v_r = node["motor.right.speed"]

        # Update odometry
        update_robot_state(v_l, v_r, dt)

        # Odom distance from origin
        dist_odom = math.hypot(robot_state["x"], robot_state["y"])

        # Stop condition 1: reached target distance
        if dist_odom >= TARGET_DIST_MM:
            print(f"\nStopping because odom distance reached "
                  f"{dist_odom:.1f} mm (target {TARGET_DIST_MM:.1f} mm).")
            break

        # Stop condition 2: safety timeout
        if now - t_start >= CALIB_MAX_DURATION:
            print(f"\nStopping because timeout ({CALIB_MAX_DURATION:.1f} s) reached.")
            break

        await client.sleep(DT_NOMINAL)

    # Stop robot
    await node.set_variables(motors(0, 0))

    # Final odom state
    dist_odom = math.hypot(robot_state["x"], robot_state["y"])
    theta_deg = robot_state["theta"] * 180.0 / math.pi

    print("\n=== Calibration result (from odometry) ===")
    print(f"Final odom pose: x={robot_state['x']:.1f} mm, "
          f"y={robot_state['y']:.1f} mm, theta={theta_deg:.1f}°")
    print(f"Odom distance travelled: {dist_odom:.1f} mm")

    # Ask user for real measured distance
    try:
        real_dist_str = input(
            "Measure the REAL distance the robot travelled (in mm) "
            "along the floor (start to end) and type it here "
            "(or just ENTER to skip): "
        ).strip()

        if real_dist_str:
            dist_real = float(real_dist_str)
            dist_est = dist_odom

            if dist_real > 1e-6:
                # Distances scale linearly with SPEED_TO_MMS:
                # dist_est ∝ SPEED_TO_MMS_old
                # We want dist_real ∝ SPEED_TO_MMS_new
                # => SPEED_TO_MMS_new = SPEED_TO_MMS_old * (dist_real / dist_est)
                SPEED_new = SPEED_TO_MMS * (dist_real / dist_est)

                print("\n=== Suggested new SPEED_TO_MMS calibration ===")
                print(f"Current SPEED_TO_MMS: {SPEED_TO_MMS:.6f} mm/s per unit")
                print(f"Odom distance estimate: {dist_est:.2f} mm")
                print(f"Real measured distance: {dist_real:.2f} mm")
                print("Suggested SPEED_TO_MMS_new = SPEED_TO_MMS_old * (dist_real / dist_est)")
                print(f"                 SPEED_TO_MMS_new ≈ {SPEED_new:.6f} mm/s per unit")
                print("\nYou can copy this SPEED_TO_MMS_new back into constants.py.")
            else:
                print("Real distance too small (or zero) to compute a meaningful factor.")
        else:
            print("No real distance entered, skipping SPEED_TO_MMS computation.")

    except ValueError:
        print("Could not parse real distance, skipping SPEED_TO_MMS computation.")


# ======================================
# MAIN ENTRY POINT
# ======================================
async def main():
    client = ClientAsync()
    node = await client.wait_for_node()
    await node.lock()

    await run_calibration(node, client)

    print("\nLinear calibration run finished.")


if __name__ == "__main__":
    asyncio.run(main())
