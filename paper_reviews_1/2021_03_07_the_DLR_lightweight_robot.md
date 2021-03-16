@def class = "journal"
@def authors = "Albu-Schaffer, A.; Haddadin, S.; Ott, Ch.; Stemmer, A.; Wimbock, T.; Hirzinger, G."
@def title = "The DLR Lightweight Robot – Design and Control Concepts for Robots in Human Environments"
@def venue = "Industrial Robot"
@def year = "2007"
@def hasmath = true
@def review = true
@def tags = ["reviews","human-robot collaboration","control","industrial robot",]
@def reviewers = ["David Yackzan"]

\toc
### Broad area/overview
This paper presents a newly developed lightweight robotic (LWR) arm with the implementation of control methods that allow for potential operation in human environments and nuanced versatility in industrial applications.

### Notation
* $x$: state, $X$: State space
* $y$: measurement, $Y$: Measurement Space
* Controller $u = g(y)$
* $L$: Set of labels
* $C_X \colon X \to L$: Classifier in state space
* $C_Y \colon Y \to L$: Classifier in measurement space
* $V$: Lyapunov function
* $W$: set of hyperplanes that parametrizes $C_X$
* $W^\Delta$: set of hyperplanes that parametrizes a robust version of $C_X$

### Specific Problem
High performance in industrial environments is typically representid by high speed, high precision functionality. In human environments, introducing this type of robot would be high risk due to the lack of adaptability to the envirenment and the potential for high speed impacts in an unpredictable setting. The DLR robot presented in this research is designed with human interaction in mind. These design aspects involve a low robot mass to payload ratio, low velocity operation speed, and extensive sensing capability such as joint torque sensing, redundant position sensing, and wrist force-torque sensing.

### Solution Ideas
Hardware overview
* Robotic arm with 7 DoF, load-weight ratio of ~1:1, totail weight < 15kg, workspace of ~1.5m
* measurements of full state of all joints completed by strain-gauge torque sensors, magneto-resistive encoders, and redundant link-side potentiometer position sensors

Control aspects for robotics acting in human environments
* collocated joint torque sensors with actuators enable robust, passivity based control approaches which is essential for operation within human environments
* A decentralized state feedback controller is implemented with motor position and velocity ($\theta,\theta'$) and joint torque and joint torque derivative ($\tau,\tau'$)
* This control strategies effectively allows for gravity compensation where the robot motors actuate to counter the joint weights and the robot can be easily manipulated

Safety Evaluation
* Impact testing with crash test dummies was completed to collect safety data. At max velocity (2m/s) the impact of the robot with the crash test dummie's head caused *very low* injury level when measured with the Head Injury Criterion (HIC).

Automatic planning of robust assembly applications using impedence control
* Another strength of this control design strategy is for use in industrial operations where assembly requires precise sensing of part to part contact
* While this robot is not as precise as more rigid industrial robots, it makes can make up for it in some manufacturing tasks by using *regions of attraction (ROA)* to push a part compliantly in the calculated direction and relying on collision feedback.


### Comments
* While the testing with the HIC scale showed that this robot is unlikely to cause serious injury to humans, robots with preventative measures implemented such as vision or lidar sensing would likely be safer to operate around humans.

### Recent Papers
* Another article addresses the topic of Human-Robot Collaboration in Manufacturing within the DLR lab more recently: *Human-Robot Collaboration in Manufacturing Applications: A Review* (https://www.mdpi.com/2218-6581/8/4/100)
