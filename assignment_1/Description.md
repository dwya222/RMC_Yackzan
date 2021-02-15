# ME 699: RMC Assignment 1 Description
### By David Yackzan

Julia version used: v1.5.3

Packages used:
* ColorTypes v0.9.1
* CoordinateTransformations v0.6.1
* GeometryTypes v0.7.10
* MeshCat v0.11.3
* MeshCatMechanisms v0.7.1
* RigidBodyDynamics v2.3.0
* StaticArrays v0.12.5

Instructions:
* Enter into the assigment_1 directory using Terminal
  - *cd assigment_1*
* Activate Julia
  - *julia*
* Go into the Julia package manager and activate the packages in the current directory
  - *]*
  - *activate .*
* Run the *startup.jl* file (
  - *include("startup.jl")*
* Wait for the visualizer to start up and then view the robot in your browser
* Follow the prompts by entering 2 vectors as instructed and view the robot moving from the starting position to the ending position in the visualizer
  - Program works best if both vectors are within the workspace of [-3.5 -3.5 -3.5] to [4 4 4])
* If you wish to run the program again after having already started everything up then run the *main.jl* file
  - *include("main.jl")*

Functions that embody solution:
* In *startup.jl*
  - *display_grabber()*
    - displays the URDF upon initialization
    - *Credit to Dr. Poonawala for function display_urdf()*
* In *main.jl* (which is included by *startup.jl*)
  - *update_grabber_state()*
    - updates the visualization of the robot in the browser as well as the started
    - *Credit to Dr. Poonawala for function update_planar_state()*
  - *distance()*
    - returns the distance between 2 input vectors
  - *get_diff()*
    - takes the current state of the robot, the desired position vector, a vector of new joint angles, and a boolean that determines whether or not the output will be visualized
    - updates the state of the robot
    - identifies coordinates of specific point on end effector in the world frame
    - returns the distance from the point to the desired ending position using *distance()*
  - *from_to()*
    - finds the distance between the current point of the end effector to the desired point using *get_diff()*
    - enters a while loop that will run as long as the distance is greater than a value of .05
    - iterates through each of the joints in a for loop, incrimenting each joint angle as long as it decreases the distance to the desired ending position
    - if the incriment does not decrease the distance, then move on to decrementing the joint angle and then the next joint
    - once the distance is below the value of .05, the program ends, and the ending position is considered acheived
