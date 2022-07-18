# Demand Sensitive Bus Dispatching System
This repository holds the code used in my dissertation.

This readme provides a brief description of the dissertation as well as the contents of the repository to run this program use main.py or test.py

---
## The Bus Bunching Problem

This phenomenon occurs when two buses who are scheduled to have a headway of  a predetermined amount of minutes but end up bunching together along the route. This  phenomenon is also referred to by other works as the Bangkok Effect (Newman, 2009),  Bus Platooning (Strathman et al., 2003) or Vehicle Pairing (Bellei and Gkoumas, 2010;   Moreira-matias et al., 2012).  The Figure titled bus bunching shows two scenarios where Scenario 1 is the  ideal scenario with little to no headway variability as the two busses progress along their  route and Scenario 2 shows two vehicles which experience bus bunching due to a high  passenger demand. There are two approaches to the bus bunching problem in similar studies. The first is that bus bunching is an effect of a bad transport infrastructure, which  is often alleviated by introducing traffic signal priority and bus lanes, and the second is that bus bunching is the main problem that must be solved, which is often alleviated by  adjusting schedules or using live control actions (Moreira-matias et al., 2012). This study focuses on the latter.
## Objectives of the Study
The aim of this study is to investigate whether a RL algorithm can help alleviate the problem of the bus bunching phenomenon using various control strategies. In order to achieve such a goal, the following objectives have been identified:

 - *Objective 1 (O1):* Simulate the bus bunching phenomenon by arranging for buses to follow a route along a ring-road with varying passenger demand using traffic simulation software (No Control) 
 - *Objective 2 (O2):* Implement Rule-Based Strategies on the same simulation and measure their effect(Rule-Based Control)
 - *Objective 3 (O3):* Apply RL Techniques in order to dynamically control the bus system 
 - *Objective 4 (O4):* Assess the performance of the agent and compare it with the results obtained with No Control and Rule-Based Control using performance metrics with respect to headways and waiting times.

## Using SUMO and TraCI to set up the Simulation Experiment

[Simulation of Urban Mobility (SUMO)](https://www.eclipse.org/sumo/) is an open source, microscopic, traffic simulation  software developed by Krajzewicz et al. (2012) that is popular in the research community.  Many researchers have made use of SUMO for their works in topics such as vehicular  communication (Rieck et al., 2010; Rondinone et al., 2013), route choice and dynamic  navigation and traffic light algorithms. SUMO offers a full suite of traffic modelling utilities  as it was designed to facilitate the modelling of large-scale road networks and cities. The  SUMO suite includes important applications, mainly; network and demand generation  and simulation.  
In order to set up the environment, SUMO four files were created using the  netedit  tool.  

 1. Network File
The first file is a network file of type  .net.xml  which includes all the edges and  junctions within the network. Since SUMO does not allow for multiple bus stops to be  created on the same edge, a total of 24 edges and 24 junctions were created. Each edge  had two lanes on which vehicles are allowed to travel and a sidewalk on which pedestrians  are allowed to walk and wait. The edges were created with a length of 500 metres. 
 2. Additional File
 The second file is an additionals file of type  .add.xml  which includes the additional  elements. In the case of this project the only additionals required were bus stops. One  bus stop was created on each edge on the outer-lanewhich connects with the sidewalk  such that passengers are able to switch from a walking stage to a waiting stage. Each bus stop was created with a length of 10 metres - allowing only one bus to board passengers  
at a time and located exactly in the middle of each edge such that between each two bus  stops, there is always 500m.  
 3. Demand File
The third file is a demand file of type  .rou.xml  which includes all elements relating  to vehicular demand. The following elements were created
	- A Vehicle Type  indicating a class of vehicles holding the in-built characteristics of a  bus. Such characteristics include; size, colour, gui-image etc.  
	-  A Route  indicating a list of consecutive edges which make up the route along with  the number of times it is to be repeated (this is done in order for the road to act as  a ring-road). 
	- 14 Vehicles  each having a unique identifier, the vehicle type set to bus, and the route  set to the  route  created prior. Each bus has a different time of departure with a time  interval of 100s between each departure. 
	- Person Flows  each having different parameters. Each person flow has a  unique identifier and a time indicatingat which it should start emitting people. The  personsPerHour  parameter was different for each flow in order to emulate the de-  mand variability created in Delgado et al. (2009). Each person  flow was created with three stages; the first is a walk stage where the persons are  instructed to walk from a pre-determined edge to the origin bus stop. Once a per-  son arrives at the bus stop, they automatically switch to a waiting stage until a bus  stops at their stop, triggering a riding stage. The riding stage indicates the destina-  tion bus stop. Finally a stop stage indicates that the person’s route ends there. It is  noteworthy that in real-world situations, not all busses that pass from a particular  stop will take the person to their intended destination. In order to make it simpler,  the assumption that any bus/line that stops at the bus stop a person is waiting at,  will take it to their intended destination is made since all busses are coded to stop  at all bus stops. 
4. The fourth and final file required for the simulation to run is the configuration file of type .sumocfg. This file points to the location of the network, demand and additional file. This is the file executed through TraCI.

## Using Open AI Gym to create custom Sumo Environment

Open AI Gym  is a toolkit that is used in reinforcement learning which comes with a  diverse collection of simulated environments. It allows users to train, test and compare  various RL algorithms. Despite including numerous built-in classic control algorithms such  as ’CartPole’ and ’MountainCar’, box2d environments such as ’LunarLandar’, and many Atari Games amongst others. A great element of Gym is that it also allows for creation of  
custom environments.
A gym environment can consist of many functions that facilitate the creation and running  of the environment but there are 3 functions which a class  must  include and these are the  initialisation function; which sets the initial state of the RL problem, the reset function  which resets the environment to its initial state when required, and the step function  which implements the action and computes the observations and the reward. 
- **The initialisation function** sets the initial state of the RL problem. Two required variables must be initialised with the names action_space and observation_space . These two variables must be of type space , which is one of Gym’s special classes. Two commonly used types in this class are Discrete and Box . 
	- Discrete is used when dealing with discrete values. Suppose that we are dealing with an action_space consisting of two actions, ’Stop’ and ’Go’, then the action space would be initialised as, self.action_space = spaces.Discrete(2) means that the action space is being created as a discrete variable which can take one of the two possible values. Box is used when dealing with real-values quantities. 
	- A Box represents the Cartesian product of n closed intervals. It can have identical bounds for each dimension or independent bounds for each dimension. Suppose
we are dealing with an observation space made up of 3 variables the number of people in the bus (with range [0,70] ), the speed at which the bus is moving (with range [0,100] ) and the temperature of the air (with range [-20,50] ) Then the observation space would be initialised as,  self.observation_space = spaces.Box(np.array([0,0,-20]), np.array([70,100,50]))
- **The reset function** restarts the environment to its initial state when required, for example at the very beginning. It returns the state which is the observations of the environment corresponding to the initial state
- **The step function** has one input parameter, the action value. This is the response received from the agent with regards to which action should be taken. The return value of this function should consist of 4 variables the state , the reward , the done boolean variable indicating true if the environment reached its end point and false otherwise and the info variable which is used to output the necessary variables’ values at that time step often used for debugging.

There are a  number of common libraries that offer various RL algorithms such as Tensorflow, Keras-RL, RLLib and D3RLPY. This project makes use of 


## State Space and Action Space for the RL Problem
The **state space** vector was defined as follows; 24 variables representing the number of passengers waiting at each of the 24 stops, the headway between the bus and its leader, the following bus' headway, the leading bus' headway, the ID of the road the bus is currently on (i.e. its location), a one-hot-encoding of the upcoming bus stop, the follower's location, the leader's location, the number of passengers on the bus and the number of passengers on the follower. 

The set of **actions** the agent is to choose from is discrete and the actions are: ’Stop and Wait’, ’Skip Stop’, ’Limit Boarding’ and ’Continue’. These actions have the following definitions in the context of this project:  
- The  Stop and Wait  Control Strategy instructs the bus to remain at the current stop  longer thus allowing the headway with its leader to increase. The time period for  this control strategy was decided to be of 30 simulation steps.  
- The  Skip Stop  Control Strategy instructs the bus which has  just  stopped at a bus  stop, to only allow people to alight the vehicle and leave immediately after. Anyone  who wished to board that bus will then have to wait for the next one, allowing the  headway between the bus and its leader to decrease.  
- The  Limit Boarding  Control Strategy compares the occupancy of the bus with that of its leader. If the bus’ occupancy is over 75%  and the following bus is less than  50%  occupied, then the bus is instructed to leave the stop and continue with its  route, allowing it to have space to pick up future passengers eliminating the issue  of becoming full-up after one stop.
- The  Continue  Action instructs the bus to ’do nothing’ and carry on with its operations


## Rule Based Control
Rule-Based Control (RBC) methods are based on a set of predefined rules to control the  system. Such methods are often used by researchers in order to compare the results obtained with those produced by their proposed strategy. In order to create a level playing  field, the same actions were used for RBC as well as Reinforcement Learning Based Control (RLBC). These are; ’Stop and Wait’, ’Limit Boarding’ and ’Skip Stop’. Each action is  associated with a particular condition and at a time, only one action can be executed.  Moreover, intervals of 50s are used to disperse the actions being taken such that the  environment has time to experience the effects of such actions.  The Rules were as follows:  
1.  If  the bus’ front headway is less than 16s,  Then  Stop and Wait extra 30s at upcoming  stop  
2.  If  the bus’ occupancy is more than 75% and the follower’s occupancy is less than  50%,  Then  Limit Boarding  
3.  If  the number of passengers at the upcoming stop are more than or equal to  1.5×  the number of passengers waiting at this stop,  Then  skip the current stop  

# main.py
This is the main file which runs the entire code. In this file one can set the control_strategy variable to either 'NC' : no control, 'RBC': rule-based-control or 'RLBC': Reinforcement learning based conrol
