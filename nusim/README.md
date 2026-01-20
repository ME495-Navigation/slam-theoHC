# NUSIM

the nusim package provides a simulator and visualizer for a SLAM system running on the turtlebot3.
It simulates a rectangular arena with cylindrical obstacles. Red objects in rviz represent "ground truth", other colors represent the system's current understanding of its surroundings.

Launchfiles:

- nusim.launch.xml
Launches the simulator along with rviz. Loads world settings from config/basic_world.yaml. The config_file argument can change the yaml used.

Parameters:
- arena_x_length: Length of the arena in the x dimension
- arena_y_length: Length of the arena in the y dimension
- obstacles.x: Array of x coords of the obstacles
- obstacles.y: Array of y coords of the obstacles
- obstacles.r: Radius of the obstacles
- x0: starting x position of the robot
- y0: starting y position of the robot
- theta0: starting heading of the robot