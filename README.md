# CarND-Path-Planning-Project

![](images/path_planning2.gif)

Self-Driving Car Engineer Nanodegree Program

## Table of Content
<!--ts-->
  * [Dependencies](#dependencies)
  * [Installation](#installation)
  * [Usage](#usage)
  * [Simulation](#simulation)
    * [Goals](#goals)
    * [Communication](#communication-between-path-planner-and-the-simulator)
    * [The highway map](#the-highway-map)
  * [Path Planner Stategy](#path-planner-stategy)
  * [C++ Classes Structue](#c-classes-structue)
    * [Vehicle Class](#vehicle-class)
    * [Trajectory Class](#trajectory-class)
    * [States](#states)
  * [Next Features](#next-features-to-be-implemented)
  * [License](#license)
  * [References](#references)
<!--te-->

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

# Installation

Just clone this repository and build it using `make build`. Assuming that you will clone this repository at `~/`:

```shell
cd ~/
git clone https://github.com/rodriguesrenato/CarND-Path-Planning.git
cd CarND-Path-Planning
make build
```

## Simulator

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:

```shell
sudo chmod u+x {simulator_file_name}
```

# Usage

This project uses Make. The Makefile has four targets:

- `build` compiles the source code and generates the `path_planning` executable.
- `format` applies ClangFormat to style the source code in [Google's C++ style](https://google.github.io/styleguide/cppguide.html).
- `debug` compiles the source code and generates an executable, including debugging symbols
- `clean` deletes the build/ directory, including all of the build artifacts
- `run` executes the `path_planning`

These are the steps to run this project:

1. Go to the project directory
2. Run `make build`. This will create an executable `path_planning` at build directory. The same applies for `make debug`.
3. Run `make run` to execute the `path_planning`.
4. Open a new terminal, go to the simulator folder and run `./{simulator_file_name}` to start it.

In the Simulator, hit `Select` button to start the simulation along with `path_planning`

# Simulation

## Goals

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the car's localization and sensor fusion data, and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, and other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Communication between Path Planner and the Simulator

For this project, Udacity provided a starter c++ code with the websocket communication framework to handle messages received from the simulator (Localization, sensor fusion and previous path data) and send back messages to it (The next path). Udacity also provided a set of conversion functions between XY and Frenet Coordinates, and the `highway_map.csv` that contains a sparse map list of waypoints around the highway.

Here is the data received from the Simulator to the C++ Program

| JSON message key | Description |
|---|---|
| ["x"] | The car's x position in map coordinates|
| ["y"] | The car's y position in map coordinates|
| ["s"] | The car's s position in frenet coordinates|
| ["d"] | The car's d position in frenet coordinates|
| ["yaw"] | The car's yaw angle in the map|
| ["speed"] | The car's speed in MPH|
| ["previous_path_x"] | The remaining previous list of x points previously given to the simulator |
| ["previous_path_y"] | The remaining previous list of y points previously given to the simulator |
| ["end_path_s"] | The previous list's last point's frenet s value|
| ["end_path_d"] | The previous list's last point's frenet d value|
| ["sensor_fusion"] | A 2d vector of all other car's attributes on the same side of the road: [car's unique ID,car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. |

Note: In the remaining previous list received, the processed points were removed.

Here is the data sent from the Simulator to the C++ Program

| JSON message key | Description |
|---|---|
| ["next_x"] | A list of x points that will be the next path |
| ["next_y"] | A list of y points that will be the next path |

The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## The highway map

The map of the highway is in `data/highway_map.txt`. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

# Path Planner Stategy 

On each message received from the simulator with new localization / sensor fusion data, the path planner will compute the best path for the car to achieve the goals. These are the main steps done by the path planner:

1. Update the current localization and sensor fusion data
2. Get a vector of all successor States from the finite state machine.
3. For each successor State, generate the correspondent Trajectory, calculate the total cost and add it to a vector of possible trajectories.
4. Get the trajectory with the lowest cost and send it back to the simulator.

## Receive data from Simulator

## Finite State Machine

The diagram below represents the finite state machine (FSM) implemented:

![](images/fsm.png)

As it isn't allowed counterflow and off road driving, the FSM automatically skips the states that would results in these conditions by testing the current lane.

## Trajectory Generation

- 

## Trajectory Cost Calculation

There were implemented 3 cost functions:

- Speed Cost
- Lane Change Cost (Inefficiency Cost)
- Collision Cost

### Speed Cost

### Lane Change Cost

### Colision



## Select Best Trajectory

## Send the best Trajectory to Simulator


# C++ Classes Structue



## States

The possible states of the finite state machine were defined as the `State` enum in the file `states.h`

There are 6 defined states:
- `R`: Ready
- `KL`: Keep Lane
- `PLCL`: Prepare Lane Change Left
- `LCL`: Lane Change Left
- `PLCR`: Prepare Lane Change Right
- `LCR`: Lane Change Right

## Trajectory Class

The Trajectory class is a simple class of public attributes that keeps all relevant informations of the path. These attributes are mainly used to calculate the trajectory costs in the `Vehicle` cost functions, to update the Vehicle class later with the selected trajectory and to store the X and Y points that will be sent back to the simulator.

## Vehicle Class

The Vehicle class is the main class responsible for unifying informations of the vehicle, sensor fusion, the road and states. The speeds given in mph are converted to meters per second before get assigned to class attributes or used by methods. 

These are the implemented methods:

Vehicle constructor
UpdateLocalizationData
UpdatePreviousPath
UpdateSensorFusion
SetCostWeights
SetChosenTrajectory
GetSuccessorStates
GenerateTrajectory
BuildTrajectory
CalculateCost
SpeedCost
LaneChangeCost
CollisionCost
CheckCollision
PredictSensorFusion
CalculateLaneIndex
CalculateBestLane
CalculateLaneSpeed
LanesToCheckSpeed
GetVehicleAhead
GetVehicleBehind
PrintStatistics

## Helper Functions

The `helpers.h` file contains conversion functions between XY and Frenet coordinates. This header file is used by the `Vehicle` class.

# Next Features to be Implemented

- Check when the last 2 positions of the previous path are the same (when speed near 0)
- Choose the best lane in PLC and LC
- Numerical validation on Vehicle Contructor (Avoid 0 values)
- Use LanesToCheckSpeed logic to the CalculateLaneIndex function to get when a vehicle from sensor fusion is changing lane
- Add VSCode profiler
- README: clang google file

```
ffmpeg -ss 485 -t 25 -i 2021-08-27\ 15-37-35.mkv -vf "fps=15,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 path_planning2.gif
```

# License

The contents of this repository are covered under the MIT License.

# References
 - Udacity Path Planning Project start guide repository. [link](https://github.com/udacity/CarND-Path-Planning-Project)
 - C++ Reference. [link](http://cplusplus.com)
 - Cubic Spline interpolation in C++: [spline.h](http://kluge.in-chemnitz.de/opensource/spline/)