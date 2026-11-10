# Camera-guided Thymio robot

Group T14 project for **Basics of Mobile Robotics** (EPFL, MA1, autumn 2025).

A Thymio robot drives to a goal on a map seen by an overhead camera. The camera finds the robot, the goal and the obstacles using ArUco markers. The robot plans a path around the obstacles, estimates its position with a filter, and follows the path.

[![Demo video](Images%20and%20Videos/thumbnail.png)](Images%20and%20Videos/demonstration.mp4)

*Click the picture to watch the demo.*

## Architecture

- `vision.py`: detects ArUco markers with OpenCV and gives positions of the robot, goal and obstacles.
- `global_nav.py`: global navigation, plans a path from the robot to the goal around obstacles.
- `filter.py`: filter that estimates the robot state from camera and robot sensor data.
- `motion_control.py`: control of the Thymio motors to follow the path.
- `main.py`: runs everything together (`traitement.py` and `constants.py` hold image-shape analysis and shared settings).

## How to run

You need Python 3, a camera, a Thymio robot and the Thymio Suite (for `tdmclient`).

```bash
pip install -r requirements.txt
python3 main.py
```

Point the camera at the map so the 4 corner markers are visible, press `s` to initialise the map, place the Thymio, then press `p` to start.

## Report

Full description of the methods and results: [Group_T14_report.ipynb](Group_T14_report.ipynb).

## Team

Ana Schwabedal, amirlh, jvillaret, Tim Kobler.
