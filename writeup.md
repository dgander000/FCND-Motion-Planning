# Write up for 3D Motion Planning Project

## Explain the Starter Code
In backyard_flyer_solution.py the waypoints are manually entered and hard coded.  See method calculate_box.   

In motion_planning.py the path is planned after arming and a new state has been added for this named PLANNING.  Method path_plan does the following to plan the path.   
- Gets the global home, global position and local position
- Gets the obstacle map
- Creates a grid for the specified altitude and safety margin around obstacles  
- Defines the grid starting point and goal location
- Runs A* search to find a path
- From this path prunes points and creates a list of waypoints
- Sends the waypoints to the autopilot and transiton to takeoff


## My Path Planning Algorithm

https://www.maps.ie/coordinates.html   
start   
37.79248   
-122.39745  

goal   
37.79173990062469   
-122.39480316638947      

