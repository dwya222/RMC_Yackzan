@def class = "journal"
@def authors = "Zoss, A.B.; Kazerooni, H.; Chu, A.;"
@def title = "Biomechanical Design of the Berkeley Lower Extremety Exoskeleton (BLEEX)"
@def venue = "IEEE; ASME Transactions on Mechatronics"
@def year = "2006"
@def hasmath = true
@def review = true
@def tags = ["reviews","exoskeleton","biomechanics","mechatronics","robotics"]
@def reviewers = ["Guan Huitao","David Yackzan"]

\toc
### Broad area/overview
This paper describes the design of an exoskeleton to be worn from hip down (lower extremety) on an individual to assist in the transportation of heavy objects. The strength of lower extremety exoskeletons is that in combination with human intelligence and control, they can aid in the transportation of heavy objects over terrain that is not navigable by wheeled robots (like stairs or rocky terrain) as well as over non-simple pathways that will take human decision and control to navigate.

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
* This design outlined in this paper focuses on providing the goal of allowing its wearer to carry significant loads without expending much effort over any terrain while considering four primary aspects: "a novel control scheme, high-powered compact power supplies, special communication protocol and electronics, and a design architecture to decrease the complexity and power consumption."

### Solution Ideas
* A control scheme was designed that allows the BLEEX exoskeleton to shadow the wearers movements, both voluntary and involuntary.
* In order to acheive this, the system uses a full dynamic model of the exoskeleton as it retrieves data from several sensors (encoders, linear accelerometers, single axis force sensors, foot switches, load distribution sensors, and an inclinometer) and actively calculates the kinematics of the skeleton.
* This exoskeleton includes 7 DoF in total:
  - 3 DoF at the hip
  - 1 DoF at the knee
  - 3 DoY at the ankle
* The control scheme consists of a power efficient method of only actuating the joints that require the most assistance during weighted locomotion, while the other joints are free or attached with a spring to allow for natural motion of the wearer and constant assistance in a preferred direction. The joints to be actuated were identified through locomotion observation trials and are as follows:
  - flexion/extension at the ankle
  - flexion/extension at the knee
  - flexion/extension at the hip
* The following equations were considered in determining the size and strength of the hydraulic actuators:
  - $T_maxpush = Ps * \frac{\pi*D_bore^2}{4}*R(\theta)$
  - $T_maxpull = Ps * \frac{\pi*D_bore^2-D_rod^2}{4}*R(\theta)$
  - Where $D_bore$ is the actuator bore diameter and $D_rod$ is the actuator rod diameter.

### Design
* Actuator Design
  - Double-acting hydraulic actuator. It has high specific power(ratio of actuator power to actuator weight), and the hydraulic fluid is incompressible leading to high-control bandwidth
  -BLEEX utilize the smallest available actuators along with a relatively low supply pressure of 6.9 MPa.(The second end-point position of the actuator can be calculated by applying angle limitations.)
 - With the supply pressure of actuator multiplied by system flow rate, the average power consumption for BLEEX to walk is 1143W compared to 165W of human gait. 14% efficiency is due to pressure drop across servo valves, this is standard for hydraulic system even though with low efficiency.
* Major Components Design
 - Overall model that simplified to emphasized major Components
  ![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/3516/33921/1618670/1618670-fig-21-source-large.gif)
 - Foot Design

   The BLEEX foot is a critical part due to its variety of functions including:
   1. Measures the location of foot's center of pressure and therefore, identifies the foot's configuration on the ground, this information is necessary for BLEEX control.
   2. Measures the load distibution on each leg, which is also a critical information for BLEEX control.
   3. Transfers the BLEEX's weight to the ground, so it must have structural integrity and long life with periodic forces
   4. One of two places of rigidly connected to body, it must be comfortable for operator.
   ![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/3516/33921/1618670/1618670-fig-22-source-large.gif)
  _The foot part has stiff heel to tranfer weight and flexible toe for comfort. Swithched detect which part of foot is contact with ground and pressure sensor detect load-distribution._
  -Other Parts from Torso to Shank
  ![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/3516/33921/1618670/1618670-fig-25-source-large.gif)
  ![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/3516/33921/1618670/1618670-fig-24-source-large.gif)
  ![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/3516/33921/1618670/1618670-fig-23-source-large.gif)
  -Final BLEEX Design
  ![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/3516/33921/1618670/1618670-fig-26-source-large.gif)

### Comments
 * The major idea of this paper is to design a lower limb exoskeleton that can closely match the kinematics and dynamics of human gait. This assumption leads to a way to design the BLEEX based on the gait data.
 * Based on this idea, the 4 actuated joints were selected, and specific actuators were selected, the actuation was designed.
 * Future Work
  - Measured curve of torque vs. angle differ from human gait and therefore, not fully match with human. Need more accurate match.
  - Reduce the consumption of overall power. Potentially by reducing weight, improve efficiency, etc.

### Recent Papers
* This paper, titled *Physical Human–Robot Interaction of a Robotic Exoskeleton By Admittance Control* outlines a more recent approach to design of a human-assistive exoskeleton: https://saa-primo.hosted.exlibrisgroup.com/primo-explore/fulldisplay?docid=TN_cdi_crossref_primary_10_1109_TIE_2018_2821649&context=PC&vid=UKY&lang=en_US&search_scope=default_scope&adaptor=primo_central_multiple_fe&tab=default_tab&query=any,contains,exoskeleton&offset=0
