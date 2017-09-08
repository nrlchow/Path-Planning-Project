# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
  
### Goals
In this project my goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data, a sparse map list of waypoints around the highway are provided. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.


Video Reference:

### Video1: 

[![Alt text](https://img.youtube.com/vi/gMI14hXdxFo/0.jpg)](https://www.youtube.com/watch?v=gMI14hXdxFo)


### Video2:

[![Alt text](https://img.youtube.com/vi/hp4Qi6O9SlY&t=31s/0.jpg)](https://www.youtube.com/watch?v=hp4Qi6O9SlY&t=31s)


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

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


# Reflection

The car is able to drive up to 7.10 miles without incident. I have used the finite state machines approach that connects discrete states for executing maneuvers in a collision-free, smooth and safe way.

# State Models
The Finite State Machines solution has taken into account below states -  

•Keep Lane and Speed
•Change Lane Left 
•Change Lane Right

These states are implemented in the main.cpp file. The main car's localization data is retrieved and "getLaneNumber(car_d)" function is used to retrieve the car's current lane. 

The "tooClose" function is used to check if it's too close to other cars and to ensure the car keeps at least 30 meters distant from the leading car. The function uses data from sensor that holds information for a list of all other cars on the same side of the road.	
	
If it finds the distance to leading car is too close, then the subsequent states are triggered. The actions perform by car lead to new states and updates current lane. Let's briefly discuss these states -  

•Keep Lane and Speed
This is a start state. In this state, the car slowly increases its speed and stay in current lane. The car continues in this this state there is no vehicle in front of it at 30 meters. If the front vehicle lower its speed then the car also decelerates and still maintains the current lane until it finds a safe gap to do lane change or avoid collision.

•Change Lane Left 
If the leading vehicle slows down, or the car's speed is greater than the front car's speed then the initiate’s lane change to left. If the distance to leading vehicle is less than 22 meters, then it stays on its lane. In this state, the car can change to left from rightmost, and center lane to left.
 
•Change Lane Right,
The same lane change rules it follows for changing lane to right. While in this state, the car can transition to center and rightmost lane from adjacent left lanes. Once the car performs the lane change it maintains it lane speed and maintains safe distance by accelerating and decelerating its speed in line with its immediate front vehicles. Once the car performs the lane change, it cannot reverse it’s this state until the lane change complete state is achieved.

The fused sensor data is used to track other cars on the same side of highway. The sensor data has provided both global and frenet coordinates values. These values are used to get lane it belongs to and check speed.50 points are generated in total for the path. Some points gets used by the simulator. I also use new points in a local car frenet coordinate system to extrapolate the path for next 90 meters. I utilize the remaining points and new points to keep the car driving smooth. I take this list of points and fit a spline in the way points and use the spline to generate a smooth trajectory, minimize the car's acceleration and jerk. Then converted the points that are lying on the spline as the points the car should follow. The trajectory is then fed to the car's controls. The latency of the simulator also get taken into account.





## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
