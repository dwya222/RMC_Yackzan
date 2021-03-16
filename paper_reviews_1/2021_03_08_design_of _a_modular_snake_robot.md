@def class = "journal"
@def authors = "Wright, C.; Johnson, A.; Peck, A.; McCord, Z.; Naaktgeboren, A.; Gianfortoni, P.; Gonzalez-Rivero, M.; Hatton, R.; Choset, H."
@def title = "Design of a Modular Snake Robot"
@def venue = "IEE"
@def year = "2007"
@def hasmath = true
@def review = true
@def tags = ["reviews","design","control", "snake robot"]
@def reviewers = ["David Yackzan"]

\toc
### Broad area/overview
This paper focuses on the mechanical and electrical design of a snake robot. The authors go into detail on various aspects of their snake robot as they have evolved from earlier/less funtional models.

### Specific Problem
The authors outline the advatages of snake robots as hyper-redundant mechanisms in environments with unpredictable terrain. They illustrate their robot climbing up pipes and over stairs to give context to the navigation abilities.
The robot discussed in the paper consists of sixteen aluminum modules all equipped with a "Super Servo" which was designed by the lab. The Super Servo is an enhanced hobby servo that incorporates sensors to measure temperature and current, as well as equipment to send and recieve information to/from an outside computer.


### Solution Ideas
* Each of the sixteen modules is able to rotate 180 degrees in a plane perpendicular to the previous module in the chain by actuation of its Super Servo. This allows for three dimensional movement of the mechanism. To enable this, the links are rotated 90 degrees from each adjacent module in the chain. To move, the robot relies on sinusoidal manipulation of its joints.
* The motion is controlled through the lab's Message Control Protocol (MCP) whereby a main computer sends positional commands and recieves temperature and current data to/from each of the modules' Super Servos.
* A controller using PID control was implemented to regulate motion as well as current draw by the motor. Current limiting effectively creates a torque limit, allowing for adjustable compliance levels with the environment.
* The authors also discuss skin design as a key factor in determining how the system interacts with its environment. It is also critical in making sure the robot's modules are protected properly from environmental debris and overheating.
* This paper concludes the discussion on the design of the snake robot by emphasizing the mechanical reliability acheived by the durable, low-friction aluminum module casings.


### Comments
* See section *IV. Electronics Architecture A. Communications* for in depth details on the communication design of the robot
* See section *III. Mechanical Overview B. Modules* and *VI. Reliability A. Mechanical* for in depth details obout the advatages of the "U" shape of the module connectors

### Recent Papers
* This paper outlines a more recent snake bot and a more specific application in which the class of robot is useful for: *Lateral Oscillation and Body Compliance Help Snakes and Snake Robots Stably Traverse Large, Smooth Obstacles* https://doi-org.ezproxy.uky.edu/10.1093/icb/icaa013
