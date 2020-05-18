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