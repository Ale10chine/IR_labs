# Logic of the RobotActionServer and CsActionClient

The two nodes implemented in `RobotActionServer.cpp` and `CsActionClient.cpp` communicate through an action interface called `Charge.action`.


## Initialization

The **Charging Station** is initialized with a random total power value within the range **80–100%**.

Similarly, the **Robot** is initialized with a random battery level within the range **5–30%**.

After this initialization, when all the callbacks embedded in the `action_client_` pointer are set up, the Charging Station sends a goal to the Robot Server containing the total power available to charge the robot.

## Charging Request Logic

The Robot accepts the charging request **only if it makes sense**. That is: if the robot’s battery level is **above 50%**, or if the **charging station can provide less than 50%** of battery charge, the action will be rejected since starting the charge would be meaningless.

In all other cases, the robot accepts the charging request and executes the `execute()` function, where the goal sent by the client is processed.


## Charging Process

The **frequency** of the processing is defined based on the charging target. So this parameter change based on the randomic initialization.  
Regardless of the initial battery level (whether the robot starts at 5% or 30%), the total charging process **always lasts 60 seconds**.

During this process, the **feedback mechanism** of the action is managed:  
at a frequency of **1 Hz**, the current battery level of the robot is continuously updated and sent as feedback to the client.


## Final Response

At the end of the charging process, the robot sends the **final result** — the last measured battery level — back to the client.  
The client then classifies the goal as **succeeded**.
