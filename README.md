# Turtlebot3 Burger Without Localization
## Features
- Path planner: A* with post processing
- Smoother: cubic Hermite spline (in use), Savitsky-Golay moving average (implemented but not in use)
- Controller: regulated pure pursuit (with curvature and proximity heuristics)
- Estimator: fusion of wheel encoders and IMU
- Bonus task: optimization of path requests from controller to planner Using ROS Service
  - As it is computationally expensive to keep requesting a path by the controller, a more efficient approach to implement a new ROS service, such that the controller prompt the planner to verify if any points along the current path have crossed into a cell on the inflation layer that has a lethal inflation cost (if the existing path is too close to an obstacle). Only if such a condition is detected, the controller will request the planner to generate a new path.

## How to Run
```shell
git clone https://github.com/joannechongeqin/turtlebot3_burger_without_localization.git
cd turtlebot3_burger_without_localization
. bd.sh
. run.sh
```
