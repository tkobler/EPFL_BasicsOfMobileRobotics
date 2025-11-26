# Local Navigation Description

## Goal of Local Navigation

The goal is to modulate the trajectory of the robot to avoid unforeseen, local obstacles. It works without a map and relies on sensors only. It is quite reactive because it doesn't require any complex sensor processing. It uses the 7 IR sensors that the robot is equipped with (5 in the front and 2 in the rear) in order to detect obstacles and reacts according to the obstacle's location. 

The IR sensors work by emitting infrared light, and then measuring how much of that light bounces back. A lot of light returning means the object is close, if the opposite is true, then the object is far away. Obstacles that are dark cause the light to be absorbed and the robot doesn’t detect them until they are very close. Cardboard covered with a white sheet of paper is a reliable choice in order to make easy to detect obstacles. 

## Implementation

Below are the most basic requirements of this implementation: 

- LOC-1: The Thymio shall locate obstacles in it's path without colliding.
- LOC-2: The Thymio shall avoid located obstacles and return to it's path when clear.

From there, one can derive lower level requirements: 

- LOC-1.1: The Thymio shall extract the position from the data acquired by the proximity sensor.
- LOC-2.1: The Thymio shall extract the 



### Avoidance strategy definition

The strategy implemented here is the **local mapless potential field**. At a high level, the potential field method makes the robot behave like a little particle in a force field. The goal is an attractive force, while any obstacles are repulsive forces. When in avoidance mode, the programm computes a resulting force vector which tells the robot in which direction to move. 

### Avoidance strategy justification

### Local Mapless Potential Field Implementation

#### Testing the sensors 

The first thing to do is to find an approximate value of the range, the update frequency, the precision/noise level of the distance sensors.

**Range:** Gives the interval of distances the sensor can measure at all. Tells *how far ahead* the Thymio can see and will thus be the determining factor in order to set a safe speed limit and in order to determine when repulsion starts.

How the test is conducted :



#### Extracting the data from the sensors



