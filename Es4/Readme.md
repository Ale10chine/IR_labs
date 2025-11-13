# Logic of the service

We have 2 nodes, one for the client and one for the server which directly implementd the service

### `burrow_client`
In the `burrow_client` we simply randomly generate n (:= the number of current apples present in the
burrow) and s (:=the maximum capability of the burrow). The client send a request to the server with
its status where the difference between s - n indicates the necessity from the burrow to understand
whther the robot can fill this lack.

### `robot_server`
In the `robot_server` there are 2 main function, 1 for manage the request of the clients and 1 for 
process data recived from the LiDAR in order to be able to make clustering on it to detect
the apples in proximity of the robot. If the number of apples are enough the server respond to the client with a true boolean since it can satisfy the request, otherwise with a false boolean.

### Processing of LiDAR msgs and detection of the r.f of the apples expressed w.r.t *base_link*
The tricky part is on the last fucntion cited, in which happens the real processing of LiDAR msgs:
