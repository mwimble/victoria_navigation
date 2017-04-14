# Strategy

The strategy module works as a goal-directed problem solver managed by **robo_magellan_node**. A goal is pushed onto a stack and each available goal solver is given a chance to decide if it can help solve the goal. Currently only one goal solver ever picks up the goal at a time, but that is not required.

As a goal solver picks up a goal, it can divide its solution into solving a series of sub problems and push new goals onto the stack.

When a goal solver picks up a goal, it response with a indication of how it handled the goal. If the goal solver succeeds or fails it removes the topmost goal from the stack and that status is captured so that any parent goal solver can react. If the response is **FATAL**, the node shuts down.

**robo_magellan_node** really just iterates over and over the list of known goal solvers and continually asks if each goal solver can solve the current goal until there are no more goals to be solved.

And example of how this would typically work is a human uses the service API to cause a **SolveRoboMagellan** goal to be pushed onto the goal stack. The **robo_magellan_node** offers each goal solver a chance to solve that goal, but only the **SolveRoboMagellan** goal solver picks it up—every other goal solver responds to the current goal with an **INACTIVE** response to indicate it can't help solve the current goal. **SolveRoboMagellan** iterates over each point it found in the YAML-given list and for each point it subdivides the problem into the sequence:

- SeekToGps
Try to get close to the waypoint.
- DiscoverCone
Try to find a cone using the camera, rotating if necessary.
- MoveToCone
Try to touch the cone.
- MoveFromCone
Backup a bit, giving room for the robot to make a move towards the next goal.

**SolveRoboMagellan** just pushes, e.g., **SeekToGps** as the current goal, asking it to move to the current waypoint in the list, and waits until **SeekToGps** succeeds or fails. If it succeeds, it pushes **DiscoverCone** as the current goal and so on until **MoveFromCone** succeeds, at which piont **SolveRoboMagellan** moves onto the next point. When there are no points left, the original goal is solved and **SolveRoboMagellan** removes its goal from the stack and the whole process is done.

If, however, one of the sub goal solvers fails, it is expected that **SolveRoboMagellan** will come up with a different recovery strategy than any that might have been employed in the subgoal.

For example. in the **MoveToCone** subgoal, it might loose sight of the cone. Especially, when it gets close enough, slight movements might move the cone from view or when the cone fills the frame of the camera, it may not pass the test for being a cone anymore (e.g., it may not be narrower at the top than the bottom). **MoveToCone** might try a local recover scheme by, say, pushing the **MoveFromCone** goal onto the stack which would probably back the robot up a bit, then see if that helps. If local recovery techniques fail for a goal solver, though, the goal **fails** and the parent goal solver should try a more complex recovery. For example, **SolveRoboMagellan** might try to move on a radial path to the cone for a bit and try to move at it from a different angle.

----
## SolveRoboMagellan
<to be done>

----
## SeekToGps
The goal solver attempts to move the robot near a GPS waypoint (see StrategyFn::**GPS_POINT**) . The point used is that which is on top of the point stack (see StrategyFn::**pushGpsPoint**).

Whenever the goal is invoked, it begins by waiting on **ConeDetector**, **Odometry**, **Fix** and **Imu** messages. When at least one of each of those messages is received, the goal proceeds.

Each goal invocation tests whether the goal waypoint has an associated cone and, if so, whether the latest **ConeDetector** message indicates that it sees the cone. If so, the goal **succeeds** as it expects the more reliable **MoveToCone** goal to take over.

Otherwise, movement towards the goal is required. For that, a heading is needed, and there are two ways to compute it:

* If the parameter **solve_using_odom** is given, the last **Odometry** message and the goal point's **x** and **y** position are used to compute a desired heading.
* Otherwise, a desired heading is computed using the last **Fix** message's latitude and longitude and the latitude and longitude of the goal point.

For each of those two computational methods, an estimated distance to the point is also computed. If that distance is less than the parameter **gps_close_distance_meters**, the goal **succeeds** as being "close enough".

The goal wants to keep the robot pointing directly at the target point. It needs to know the current heading of the robot, and it can determine that in one of two ways:

* If the **use_imu** parameter is _true_, it is used as the current heading. The value of the **magnetic_declination** parameter is added to the imu heading to get a true heading.
* Otherwise the last **Odometry** message's `pose.pose.orientation` is used as the current heading. Note that the odometry heading is expected to already reflect a true heading—no magnetic declination is added.

A goal yaw is computed to be the smallest angle between the current heading and the desired goal heading. If the goal yaw is less than the parameter **goal_yaw_degrees_delta_threshold**, the current heading is deemed "close enough" for the moment and no yaw correction is made. Otherwise, the robot is commanded to yaw in the desired direction a bit.The `cmd_vel.angular.z` component is set to the parameter value **yaw_turn_radians_per_sec** with the appropriate sign, and the `cmd_vel.linear.x` value is set to half of the **linear_move_meters_per_sec** parameter value. 

The "close enough" test is made to prevent the robot from spending all of its time making relatively slow movements that are unlikely to ever result in the heading being exactlly correct. Adding an x component to the turn command results in a smoother turn. although it does prevent a "turn in place". Since the goal point is some distance away, "turn in place" should not be required.

If no corrected yaw command is needed, the robot is commanded to move forward by issuing a command with `cmd_vel.angular.z` equal to zero and `cmd_vel.linear.x` equal to the parameter value **linear_move_meters_per_sec**.

----
## DiscoverCone
<to be done>
----
## MoveToCone
<to be done>
----
## MoveFromCone

----
## StrategyFn
This class is the parent class of every goal solver. It contains a few class globals that are used by all goal solvers, such as the list of GPS points to be found, the goal stack, a related GPS point stack and the last goal solver response (only the **SUCCESS** and **FAILED** responses are captured).

- typedef struct **GPS_POINT**
Each waypoint to be visited by the robot is converted from its YAML form into an enhanced structure that holds not only the original GPS latitude and longitude and whether or not the waypoint is expected to have a cone, but also the equivalent absolute x and y point corresponding to that latitude and longitude, with the assumption that x=0 and y=0 is where the robot was positioned at the start (viz., the latitude and longitude in the last **Fix** message at the first time the GPS_POINT was computed). Also, the true bearing (a.k.a., heading) and distance from the previous point is computed. Remember that the first point in the list is **NOT** the starting point of the robot. The starting point is implicityly the **Fix** message latitude and longitude at the time the **GPS_POINT** is created. So the first **GPS_POINT** bearing and distance is that from the starting point. After the first GPS_POINT is computed and pushed onto the list of waypoints, each succeeding point's bearing and distance is that from the previous point in the list.
~~~c++
	typedef struct GPS_POINT {
		double latitude;
		double longitude;
		bool has_cone;
		double x;			// Position relative to start.
		double y;			// Position relative to start.
		double bearing;		// Position bearing from start
		double distance;	// Distance to point.
	} GPS_POINT;
~~~

- enum **RESULT_T**
As each goal solver is asked to solve the current goal, it responds with a result that indicates how it did. The most common responses are:
	* **INACTIVE**
  	The goal solver doesn't see how it can solve the goal.
	* **RUNNING**
  	The goal solver is actively and incrementally trying to solve the goal.
  	* **SUCCESS**
  	The goal solver successfully solved the goal and removed the goal from the stack.
  	* ** FAILED**
  	The goal solver was unable to solve the goal, even with its own recovery strategies and removed the goal from the stack.
~~~C++
	enum RESULT_T {
		UNUSED_START = 0,	// Do not use, must be first element.
		FAILED,				// Strategy failed, do not continue.
		FATAL,				// Something is fatally wrong.
		INACTIVE,			// Strategy is not active.
		RUNNING,			// Strategy is in progress.
		SUCCESS,			// Strategy succeeded, continue on.
		UNUSED_END			// Do not use, must be last element.
	};
~~~

- static void **pushGpsPoint**(const **GPS_POINT**& gps_point)
GPS points are an ordered list of points to be traversed by the strategy module. They are setup by invoking **pushGpsPoint**, where the first point pushed is the first point to be found and the last point pushed is the end point (goal)
