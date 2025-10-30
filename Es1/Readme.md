# Logic of the robot
The program simulates an autonomous system based on two nodes: one that emulates the behavior of a vacuum cleaner robot and another that emulates the behavior of a charging station.

1. ### The Charging Station Node
The charging station is implemented as a simple "reader node." It is limited to monitoring the communication topic, receiving the robot's current status, and printing the information to the console.

2. ### The Robot Node 
The robot's logic operates on three concurrent processes (managed by independent timers) to perform its functions simultaneously:

- A. *Battery Degradation Simulation*:
A simulation of the battery degradation starts instantly when the robot node is initialized. This runs on a dedicated, high-frequency timer to ensure continuous monitoring and power consumption modeling.

- B. *Action Selection*:
Every 10 seconds, the robot selects a new random action (i.e., a new room to clean). The time dedicated to cleaning and movement is considered "instantaneous" or is embedded within the 10-seconds interval between one action selection and the next.

- C. *Status Update*:
The third process is responsible for broadcasting the robot's current status (location, battery level) with a frequency of 0.5 Hz (every 0.2 seconds) on the designated topic.

The node employs logic to check the battery status: if the battery level drops below a set threshold, the node stops the core activities (Action Selection and Battery Degradation simulation) by directly switching off the node.