@def class = "journal"
@def authors = "Lathuiliere, S.; Masse, B.; Masejo, P.; Horaud, R."
@def title = "Neural Network Based Reinforcement Learning for Audio-Visual Gaze Control in Human-Robot Interaction"
@def venue = "arXiv"
@def year = "2018"
@def hasmath = true
@def review = true
@def tags = ["reviews","reinforcement","learning","nearal network","human-robot interaction"]
@def reviewers = ["David Yackzan"]

\toc
### Broad area/overview
This paper outlines a research project to design a robot capable of teaching itself to gaze at a speaker in a room through reinforcement learning using audio and visual input. The robot used has 2 degrees of freedom: pan and yaw of the head. This motion determines the field of vision as the cameras are located in the head. The microphones for audio input are not located in the head so their "field of vision" is not impacted by the actuation of the head.
The authors present the adaptability of a reinforcement approach to the gaze issue as a significant advantage over similar research that has been conducted using other methods.This approach is very humanistic considering the method of audio-visual input (like a human's eyes and ears) in combination with the method of reinforcement learning (like how humans learn through rewards).

### Notation
* $R_t$: The reward at a given time
* $F_t$: The number of observed faces at a given time
* $\Sigma$: Binary variable representing whether or not speaker is within the field of vision
* $\alpha$: Reward weight variable of $\Sigma$

### Specific Problem
Rather than using other training methods, the reinforcement learning solution to gaze control requires no supervision. It only requires an accurate method of determining whether or not a decision has the intended outcome, in other words appropriately determining a reward. Reinforcement learning is implemented by outlining a function (called a policy) that will complete an action based on the optimization of a reward function.
In this specific case the reward should be given when the robot actuates its head properly to where the speaker is in its field of vision.


### Solution Ideas
* At each time step, the robot gathers data from its motors, cameras, and microphones and completes an action based on the policy function that is injected with these inputs. The action that the policy function returns in this case is the actuation of the motors controlling the head position and field of view.
* Visual observations of people are made from the camera input using a landmark model whereby the number of people in view are determined by the number of human-features (or landmarks) are detected. These features can be noses, eyes, ears, shoes, hands, legs, etc. This allows for the system to identify observed people as well as observed faces ($F_t$).
* The audio data from the microphones is taken in and analyzed to render an audio map matrix locating where the sound is in reference to the robot. If the speaker is within the field of vision, the binary variable $\Sigma$ is set to 1, otherwise it is 0.
* The reward is then determined by: $R_t = F_t+1 + \alpha\Sigma_t+1$
* The policy is a function of the reward $R_t$ and the joint positions of the head actuators.
* It would take an extensive amount of time for the robot to be trained from scratch on startup each time, so the authors outline how the robot pre-trains on a simulated environment at first. This simulated environment presents a processing speed advantage as the model does not require audio and visual data to train, only parameters outlined in the policy, which drastically decreases training time.
* In order to test the different models, simulated environments were randomly generated and the runs were scored based on the average reward they acheived.
* The network that acheived the best average reward was called *LFNet*, a late fusion strategy.


### Comments
* The article presents on interesting difficulty that comes when comparing RL models:
  -When comparing interpretation of data, even if the data is the same, what the robot actually interprets from the given data set will vary depending on the actions it has taken for that specific model. For example, if the robot turns his head $x$ degrees for one model and $y$ degrees for the other model, then the interpreted visual data set will be different, even though as a whole the data set may still be the same.
* The authors do not describe how the robot distinguishes between ambient noise and voices, so if ambient noise is not accounted for and this testing was done in an acoustically controlled environment, then this could present a flaw of the design.
* In depth technical details about the training algorithms are provided in section 3, pages 5-6 of the article.
