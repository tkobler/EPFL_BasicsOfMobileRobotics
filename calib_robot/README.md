# Robot Calibration Tools

This directory contains calibration scripts for the Thymio robot used in the BMR project.

## Scripts

### `calibrate_lin.py`
Calibrates the linear speed parameter `SPEED_TO_MMS` by running the robot in a straight line and comparing odometry estimates with real measurements.

**Usage:**
```bash
python3 calibrate_lin.py
```

### `calibration90.py`
Calibrates the wheelbase parameter `L` by running the robot in a turning motion and comparing odometry estimates with real measurements.

**Usage:**
```bash
python3 calibration90.py
```

### `variance.py`
Determines the Q matrix (process noise covariance) for the Kalman filter by running multiple movement trials and calculating variances.

**Usage:**
```bash
python3 variance.py
```

**What it does:**
- Runs 10 trials of straight-line movement (3 seconds each)
- After each trial, prompts the user to measure the actual position (x, y, theta)
- Compares odometry estimates with real measurements
- Calculates variances for x, y, theta, and wheel speeds
- Outputs the Q matrix and suggested values for `constants.py`

**Requirements:**
- Thymio robot connected via TDM
- Measuring tools: ruler/tape measure and protractor
- Flat, clear surface for robot movement
- Marking pen to mark initial position

**Example output:**
```
Variances calculées:
  var(x)     = 0.123456 mm²
  var(y)     = 0.234567 mm²
  var(theta) = 0.012345 rad²
  var(speed_left)  = 1.234567
  var(speed_right) = 1.345678
  var(speed_avg)   = 1.290123

Valeurs à copier dans constants.py:
X_MOVE_UNCERT = 0.123456  # uncertainty in x direction (movement noise)
Y_MOVE_UNCERT = 0.234567  # uncertainty in y direction (movement noise)
THETA_MOVE_UNCERT = 0.012345  # uncertainty in theta (movement noise)
SPEED_MOVE_UNCERT = 1.290123  # uncertainty in wheel speed (movement noise)
```

## Prerequisites

All scripts require:
- Python 3.x
- numpy
- tdmclient

Install dependencies:
```bash
pip install numpy tdmclient
```

## Notes

- Make sure the Thymio robot is connected before running any script
- Place the robot on a flat surface with enough space
- Mark the initial position to ensure consistent measurements
- Take careful measurements for accurate calibration results
