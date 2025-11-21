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
SPEED_TO_MMS = 0.032769  # mm/s per Thymio speed unit (your current value)
L = 122.7                             # current guess for wheelbase in mm

# Calibration motion
CALIB_LEFT_CMD = 200      # motor.left.target
CALIB_RIGHT_CMD = 0       # motor.right.target  (0 gives a strong right turn)
TARGET_ANGLE_DEG = 90.0   # stop when |theta_odom| >= this (deg)
CALIB_MAX_DURATION = 6.0  # safety timeout in seconds

DT_NOMINAL = 0.05         # nominal sleep between iterations (s)


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
    omega = (v_r - v_l) / L    # rad/s

    # Integrate position
    robot_state["x"] += v * math.cos(robot_state["theta"]) * dt
    robot_state["y"] += v * math.sin(robot_state["theta"]) * dt
    robot_state["theta"] += omega * dt

    # Normalize angle to [-pi, pi]
    robot_state["theta"] = (robot_state["theta"] + np.pi) % (2 * np.pi) - np.pi


def motors(left, right):
    """Helper to format motor commands."""
    return {
        "motor.left.target": [int(left)],
        "motor.right.target": [int(right)],
    }


# ======================================
# Async calibration routine
# ======================================
async def run_calibration(node, client):
    # Ensure we see motor speeds
    await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})

    # Reset odometry
    robot_state["x"] = 0.0
    robot_state["y"] = 0.0
    robot_state["theta"] = 0.0

    target_angle_rad = math.radians(TARGET_ANGLE_DEG)

    print("=== Thymio turning calibration ===")
    print(f"Current L guess: {L:.2f} mm")
    print(f"Motion: left={CALIB_LEFT_CMD}, right={CALIB_RIGHT_CMD}")
    print(f"Stop when |theta_odom| >= {TARGET_ANGLE_DEG:.1f}° "
          f"or after {CALIB_MAX_DURATION:.1f} s (timeout).")
    input("Place the robot on the floor, align it carefully, "
          "then press ENTER to start the calibration motion...")

    # Start motion
    await node.set_variables(motors(CALIB_LEFT_CMD, CALIB_RIGHT_CMD))

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

        theta_deg = robot_state["theta"] * 180.0 / math.pi

        # Stop condition 1: reached target angle (in odometry)
        if abs(robot_state["theta"]) >= target_angle_rad:
            print(f"\nStopping because |theta_odom| reached "
                  f"{abs(theta_deg):.1f}° (target {TARGET_ANGLE_DEG:.1f}°).")
            break

        # Stop condition 2: safety timeout
        if now - t_start >= CALIB_MAX_DURATION:
            print(f"\nStopping because timeout ({CALIB_MAX_DURATION:.1f} s) reached.")
            break

        await client.sleep(DT_NOMINAL)

    # Stop robot
    await node.set_variables(motors(0, 0))

    # Report odometry-based rotation
    theta_final = robot_state["theta"]   # rad
    theta_deg = theta_final * 180.0 / math.pi

    print("\n=== Calibration result (from odometry) ===")
    print(f"Final odom pose: x={robot_state['x']:.1f} mm, "
          f"y={robot_state['y']:.1f} mm, theta={theta_deg:.1f}°")

    # Ask user for real measured angle
    try:
        real_deg_str = input(
            "Measure the REAL angle the robot turned (in degrees, "
            "positive = counter-clockwise, negative = clockwise), "
            "and type it here (or just ENTER to skip): "
        ).strip()

        if real_deg_str:
            theta_real_deg = float(real_deg_str)

            # Use magnitudes to avoid sign fuss
            theta_est_abs = abs(theta_deg)
            theta_real_abs = abs(theta_real_deg)

            if theta_real_abs > 1e-6:
                # Relation: theta_est / theta_true = L_true / L_est
                # => L_true = L_est * theta_est / theta_true
                L_new = L * (theta_est_abs / theta_real_abs)

                print("\n=== Suggested new L calibration ===")
                print(f"Current L: {L:.2f} mm")
                print(f"Odom estimated angle: {theta_deg:.2f}°")
                print(f"Real measured angle: {theta_real_deg:.2f}°")
                print("Suggested L_new = L_old * (theta_est / theta_real)")
                print(f"           L_new ≈ {L_new:.2f} mm")
                print("\nYou can copy this L_new back into constants.py.")
            else:
                print("Real angle too small (or zero) to compute a meaningful L.")
        else:
            print("No real angle entered, skipping L computation.")

    except ValueError:
        print("Could not parse real angle, skipping L computation.")


# ======================================
# MAIN ENTRY POINT
# ======================================
async def main():
    client = ClientAsync()
    node = await client.wait_for_node()
    await node.lock()

    await run_calibration(node, client)

    print("\nCalibration run finished.")


if __name__ == "__main__":
    asyncio.run(main())
