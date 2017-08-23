# PATH PLANNING PROJECT

##### JAKRIN JUANGBHANICH

The goal of this project is to create a system which drives a simulated car along a highway with traffic. The highway waypoints are provided, as well as the current state of the car, and the sensor fusion data of other vehicles on the road. The goal is to make our ego car travel along the highway as safely, quickly, and smoothly as possible.



### OVERVIEW

I approached this project after having watched the Udacity walkthrough video (which I used as a starting point). I first made the car naively travel along the lane, and then added additional 'modules' to the car, to handle things like safety, and lane changing.



### MOTION

The car's motion is controlled exclusively by `path_module.cpp` which defines the following values:

```C++
double spline_increaser_ = 37.5;
double spline_node_count_ = 3;
double waypoint_node_count_ = 35;
double lane_change_rate_ = 0.02;
double acceleration_ = 0.275;
```
This uses the same logic as outlined in the video, generating a spline anchor then extending it out against the desired waypoints. The car's path is then generated along the spline, and appended to the previous remaining path (for smoothness).

The waypoints node count (number of 'pac-man' dots to generate) I set at **35**, because I felt that gave the car a better ability to react to external factors on the road.

The path module is fed a **target velocity** from the car, which is then used to decide whether it should accelerate or decelerate.  One thing I'd like to improve for the future is to include a smoother way to accelerate, and support for different acceleration values (for example emergency brakes).



### LANE ANALYSIS

In order to help the car make decisions about its target velocity or target lane, I used `lane_analyzer.cpp`.  Each lane has a bunch of associated cost objects, which is essentially running an input number against a threshold (user defined),  using a sigmoid function (a trick picked up in the Deep Learning lessons!) and a weight to generate a final cost. The idea is that I want a very clear and easy way to define the cost of being in a particular lane.

##### LANE COSTS

```C++
cost_forward_speed_.ApplyRange(50, 25).ApplyWeight(5);
cost_forward_distance_.ApplyRange(70, 20).ApplyWeight(8);
cost_back_distance_.ApplyRange(10, 0).ApplyWeight(50);
cost_back_speed_.ApplyRange(-10, 0).ApplyWeight(50);
```
In this snippet, the cost has a range, and a weight. For example, the `cost_forward_distance_` measures the distance in meters between my car and the nearest one in front (in that lane). If the distance is 50m or more, then the cost approaches 0. If it is 25 or less, the cost approaches 1 (using a sigmoid function). The weight is then multiplied on, so that I can easily define how costs are to be measured against one another.

The costs involved are:

* The distance between my car and the one in front.
* The speed of the car in front.
* If there is a car to the side (`cost_back_distance_`).
* If there is a car behind me, but is travelling faster than we are.

*Additionally, I set the middle lane to have a lower base cost than the two edge lanes. This is because, given equal circumstances, I prefer the car to be in the middle. This simply gives it more options when there is heavy traffic.*

##### LANE CHANGING

```c++
float lane_change_min_cost_ = 0.5;
float lane_change_max_cost_ = 8.5;
float lane_change_min_speed_ = 37.5;
```
For the car to change lanes, the cost advantage of the adjacent lane must be higher than `lane_change_min_cost_`. The overall cost value of that lane must also be less than `lane_change_max_cost_`  to prevent the car changing into a dangerous situation, even if it has a lower cost than the current lane. Finally, the car must be travelling faster than `lane_change_min_speed_` mph, because it is dangerous to change lanes when the car is travelling too far below the speed limit.

Once the car has decided to change lanes, it will begin steering to the target lane. During this time, it will not consider any other lane changes.

##### NOTES ON LANE CHANGING

Upon observing the simulation, lane changing is the leading cause for collisions. I would notice my car would sometimes change too slowly, or reacts poorly when a vehicle in front cuts it off. A more conservative approach to lane changing also means the car is more likely to be 'stuck' and unable to shift out of a slow lane because it doesn't want to take the risk.

Also, a noted improvement for the lane change module would be to have a system to 'overtake' slower lanes. This is usually the case when the car is keeping distance behind the vehicle in front, but is blocked to its immediate side by another vehicle. My car should be smart enough to know that it can accelerate, or slow down, to escape the block and go to a faster lane.



### SAFETY

Aside from the costs functions of the lanes, the car has two additional safety features. Firstly, it will maintain a distance of `double speed_limit_distance_ = 27` meters behind the car directly in front when driving normally.

Secondly, the `safety_module.cpp` will quickly slow down the car if it somehow comes within 10m distance of another car (usually this happens when our car or another car changes lanes quickly).



### CONCLUSION

My best recording distance without incident was 70 miles. If I were to work on this further, I'd improve the simulator to have more output (for example, incident logs, replays), and give my car the ability to indicate left or right. I'd want to reduce the incident to zero, and write a module to help it escape from a 'blocking' situation. I'd also probably want to re-write the path planning (spline) part from the ground up, so that I actually understand the math behind making smooth paths.