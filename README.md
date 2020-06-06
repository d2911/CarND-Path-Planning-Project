# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Reflection

Using Sensor Fusion data, Frenet Co-ordiante, Prediction, Behaviour Planning and trajectory generation to help Car in simulator to self drive on it own with a speed limit of 50mph and is able to switch lane when it is required and safe to do it.

### Prediction

Position, velocity and various information of vehcile around our car is available in sensor fusion data. With this data in each lane future position and velocity of each car in close proximity (less than 30m) are determined. Cost is assignes to each lane based on how close each vehicle to our vehcile in future point (closer the other vehicle higher the cost in that lane). 

Deceleration and acceleration of our car is determined here. Based on the distance between our veicle to one in the front, if it is 30m vehicle speed is reduced slowly but it is too close (15m) vehicle speed is reduced very fast. Once the distance increases velocity is increased slowly if the current vehicle speed is greater than 20mph and velocity is increased very fast if the current velocity is less than 20mph. This takes care of collison avoidance, additionally spped linit of 50mph is considered here. More score of improvement is there.

In each lane, velocity of first vehicle in front of our car is determined to find the fastest lane.

### Behaviour Planning

Finite state machine is used to implement Behaviour planning, possible states are as per below enum, READY, Keep Lane, Prepare Lane Change Right, Prepare Lane change Left, Lane Change Left and Lane Change Right.

`enum FSMState{READY=1, KEEPLANE, PLCR, PLCL, LCR, LCL};`

In the state machine, cost based on distance between vehcile and velocity of vehicle in each lane determined in prediction step are used to make state transition. Left lane, center lane and right lane are considered as 0, 1 and 2 respectively in the algorithm.

__State READY:__  
   This is the default state when program starts.  
   __Transition__  
      1. KEEPLANE : every time READY state is reached, state transition to KEEPLANE is triggered.  
  
__State KEEPLANE:__  
   When vehicle is close enough to vehcile in front, this state will look for efficient lane change.  
   __Transition__  
      1. PLCR: When curent lane of our car is left or center and risk in right lane [cost of lane determined in previous step] is less than a threshold.  
      2. PLCL: When curent lane of our car is right or center and risk in left lane [cost of lane determined in previous step] is less than a threshold.  
      Note: when car is in center lane right turn or left turn happens based on where cost is less.  
  
__State PLCR:__  
   This is intermediate state before lane change to right. Before initiating lane change velocity of vehicle in each lane, determined in prediction, is used to check the fastest lane. If current lane is fastest lane change is aborthed and our car will continue in the current lane.  
   __Transition__  
      1. KEEPLANE : When traffic in current lane is faster than right lane.  
      2. LCR: When traffic in right lane is faster than current lane and velocity of car is atlease 30mph.  

__State PLCL:__  
   This is intermediate state before lane change to left. Before initiating lane change velocity of vehicle in each lane, determined in prediction, is used to check the fastest lane. If current lane is fastest lane change is aborthed and our car will continue in the current lane.  
   __Transition__  
      1. KEEPLANE : When traffic in current lane is faster than left lane.  
      2. LCR: When traffic in left lane is faster than current lane and velocity of car is atlease 30mph.  

__State LCR:__  
   Right lane change is triggered and stays in this state for 10 count.  
   __Transition__  
      1. KEEPLANE : After 10 count in this state, state change is triggered.  
      
__State LCL:__  
   Left lane change is triggered and stays in this state for 10 count.  
   __Transition__  
      1. KEEPLANE : After 10 count in this state, state change is triggered.  

### Trajactory Generation

spline library is used to generate smooth trajectory for lane changes. Refer tips section for more details on spline. Five points are constructed, last two points from previous path and three more points are determined from future ta gap of 30m, 60m and 90m. Using these five points spline is constructed, using spline and 50 points in x direction are used to determine points in x,y co-ordinate.  

### Test Run on Simulator

  
  

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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
