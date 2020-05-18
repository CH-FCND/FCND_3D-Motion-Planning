## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
The motion_planning.py code accomplishes:

It does so by starting off with setting the main function when motion_plannin.py is launched in python.

    An argumentParser object is created to handle the communication path to the drone and easily pass the arguments to the connection. 
    
    The connection between the drone and the controller is setup using a MAVLink protocol.

    A Motion Planning object is instantiated by passing the connection to the MotionPlanning class constructor.

    The thread is slept to ensure that the MotionPlanning Object is created properly and then the start method is called.


When the MotionPlanning object is instantiated the ocnnection is passed to the constructor which relies on inheriting from the Dron class and setting up the connection.

The MotionPlanningConstructor creates:
    - a target position in the form of a 3 element numpy array
    - an empty list / queue of waypoints is declared
    - a boolean is set for whether the drone is in a mission or not
    - the check_state variable is declared but doesn't appear to be used
    - the flight_state enumeration is declared which represents the physical state of the drone and is set to MANUAL initially to ensure control can be established
    - Three callback functions are registered to be called when there is an updated to the following:
        - LOCAL_POSITION - calls local_position_callback
        - LOCAL_VELOCITY - calls velocity_callback
        - STATE - calls state_callback

The Start method sets up a logging file, starts the drone and closes the log file based on functions within the Udacidrone class we have inherited from.

With inheritance of many functions out of th way, this code boils down to the three callback functions:


1) local_position_callback - flight path controller
        - when the local position message is recieved the callback checks if the state is in the TAKEOFF state or in the WAYPOINT state.
        
        A) If the drone is in the TAKEOFF state and the altitude is 95% of the target altitude the drone assumes that the takeoff was successful and calls the waypoint_transition method.  The waypoint_transition method which sets the flight_state to WAYPOINT following, removes the first waypoint from the waypoints queue and sets that to the target_position.  The inherited UDACIDRONE method cmd_position is fed the north, east, altitude and heading popped from the waypoint list.

        B) If the drone is in the WAYPOINT following state already, the linear distance between the target_position and the drone's local position is calculated.  If it is less than 1.0 meters (UDACIDRONE is in meters) it confirms that the waypoint has been achieved and checks whether there are more waypoints and transitions to the next waypoint if available, or waits for the velocity to be less than 1 m/s around the target landing waypoint and calls the landing_transition method.
            - If the landing_transition method is called because the drone is at the last waypoint, the flight_state is transisiton to the LANDING state and the land() command is issued calling the UDACIDRONE land() method
            - The disarming_transition method sets the state to DISARMING, calls the disarm() and the release_control() methods which disarms and sets the drone to manual.



2) velocity_callback - controls the landing to disarming transition
    - when the velocity is received the drone state is checked against the LANDING state or not.  If the drone is in the landing state the GPS altitude is checked against the GPS home position altitude.  When this is less than 0.1 the control turns to the local_position measurement.  When the local_position is less than 0.01 m the drone is transitioned to the DISARMING state by calling the disarming_transition() method


3) state_callback - flight path planning functionality and state transitioning.
    - When the state is received and the vehicle is in a mission, the State is checked.  
        - If the drone state is MANUAL, the arming_transition method is called. This transitions the state to ARMING, calls the UDACIDRONE arm() and trake_control() methods to start the drone.
        - If the drone state is ARMING it waits for the armed boolean to indicate it's ready and then calls the path_plan() method.
            - The path_plan() method does a signifcant amount of work:
                - The state gets transitioned to PLANNING to execute a planning algorithm. 
                It sets the target altitude and safety distance from objects and searches for a plan.  This is where the work needs to be done for this project.
                - Currently, the target_position is set to the target altitude variable, the global home, global position and local position are printed out.
                - The obstacle map is loaded in and the create_grid method is called to build the 3d grid of obstacles from the map, target_altitude and safety distance.
                    - create_grid method in planning_utils.py:
                        - The create_grid method extracts the size of the obstacle map from the obstacle data in the form of a 2-D numpy array of zeros.
                        - The numpy zeros array is then populated with obstacles by placing a 1 in each obstructed grid cell
                        - The grid of obstacles/free space and the grid corner is returned from the function 
                - The grid_start position is instantiated to the corner of the map and the grid_goal position is set to be (+10,+10) grid cells away.
                the path is determined using the a_star method.
                    - a_star method in planning_utils.py:
                        - The a_star method is the same method that was used in the course lessons, expanding nodes to find a lowest cost solution with the help of a heuristic, the grid and a start and goal location.
                    - heuristic method is a linear distance between two points (the evaluated position and the goal state)
                - The waypoints are then built from the path and the start position offset and assigned to the drone objects waypoints.
                - The send_waypoints method is called to visualize the waypoints in the simulator.

        - If the drone is in the PLANNING State, the drone transitions to TAKEOFF by calling the trakeoff_transition() method which sets a target altitude to complete the trakeoff transition in the local_position_callback

        - If the drone is in the DISARMING state the drone waits for the armed and guided booleans to be false to call the manual_transition() method.  The manual_transition() method sets the State to MANUAL calls the UDACIDRONE stop function which ends the connection to the controller and sets the in_mission flag to False allowing the program to terminate properly.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file to extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. The first line contains characters in the form of letters and numbers.  The position is read in as a list and each of the two items in the list are split and the numbers are converted into floats.  The global home is set in terms of the GPS / geodetic frame.

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

The local position global position in geodetic coordinates is set and converted to local cartesian coordinates using the global_to_local function.


#### 3. Set grid start position from local position
The grid_start position is set using the drone's cartesian coordinates and knowing where the corner of the map is.  Adjusting the offset of the drone to the map corner sets the position of the drone to move in the cartesian frame.

#### 4. Set grid goal position from geodetic coords
Selecting a longitude and latitude by manually navigating to a location on the map and recording the coordinates allows the user to input these coordinates into the global_to_local method and get the cartesian coordinates.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.
This has been done by adding to the ENUM state NorthEast, NorthWest, SouthWest and SouthEast movements with costs of square-root(2) 

#### 6. Cull waypoints 
By reusing the code written for the culling method from course material and importing into the motion_planning function I pass the a_star path into the function and remove coordinates that are collinear.  The function steps through each point to see if it is collinear.  By increasing the epsilon in the collinearity check function we can smooth out more points.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


