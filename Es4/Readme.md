
# Logic of the Service and package for the robot with LiDAR sensor

We have 2 nodes, one for the client and one for the server, which directly implements the service.

### `burrow_client`
In the `burrow_client`, we simply randomly generate $n$ ($:=$ the number of current apples present in the burrow) and $s$ ($:=$ the maximum capacity of the burrow). The client sends a request to the server with its status, where the difference $s - n$ indicates the necessity from the burrow to understand whether the robot can fill this lack.

### `robot_server`
In the `robot_server`, there are 2 main functions: 1 for managing the requests of the clients and 1 for processing data received from the LiDAR in order to be able to perform clustering to detect the apples in proximity of the robot. If the number of apples is enough, the server responds to the client with a true boolean, since it can satisfy the request, otherwise with a false boolean.

### Processing of LiDAR messages and detection of the r.f. of the apples expressed w.r.t. *base_link*
The tricky part is the last function cited, in which the real processing of LiDAR messages happens. Inside the function `process_lidar_data_callback()`, we first parse the data of the LiDAR messages to retrieve the distances of each point of the 2D point cloud. Then, through a custom clustering algorithm based on a simple principle, it is possible to detect near objects and walls that are surrounding the robot.
The clustering is done directly on the raw data of the distances of the points: when we find patterns of discontinuity, it is possible to understand if something of tiny dimension is near to the robot. From this clustering, it is then possible to further process the clusters obtained in order to place the r.f. into each object detected. To do so, we select the nearest point of the cluster to the LiDAR r.f., and through some trigonometry processing, it is possible to individuate the $X, Y$ coordinate w.r.t. `base_scan`, maintaining the same orientation and setting $Z = 0$, since the points are placed on the $XY$ plane of the LiDAR. At the end, through the `tf2` library, it is possible to express these r.frames w.r.t. `base_link`, and sending them on the `/apples` topic, it is therefore possible to visualize them in real time on Rviz2.

The clustering approach used is robust for this kind of application, in which we were asked to only detect objects that surround our robot. Clearly, more sophisticated approaches, such as computer vision based ones, could be more adaptive to other types of solutions, for example, in circumstances where the robot is moving in the environment.

### Clustering Visualization

An important notice must be made regarding the visual aspect, to **effectively see how the clustering is performed** and subsequently **how the *r.frames* are placed on the apples**.

### RViz Instructions

To visualize the apples' *r.f.* (reference frames) in real time:

1.  Open **RViz**.
2.  In the **Displays** section, click **Add**.
3.  Select **By topic**.
4.  Choose the topic: `/apples`.
5.  Select the message type: **Pose Array**.

In this manner, after launching the server, the apples' *r.f.* will **appear in real time** in the viewer. Is also suggested to deselect all frames except from base_link and base_scan, which are the only ones of our concern.

### How to execute Es4
Enter in the ws directory and then follow the istruction below:

Make executable the bash script to execute all the necessary programs:
 ```
 chmod +x launch_exercise_4.sh

 ```
Launch the script with:

 ```
./launch_exercise_4.sh
```
After this, you can follow the written instructions to **plot the apples' r.frames in real time** thanks to the clustering. You can also **launch multiple clients** and iterate the request procedure to the server, which will remain online until you close it. To launch a client use:
```
ros2 run twenty_four_ex4 burrow_client
```
