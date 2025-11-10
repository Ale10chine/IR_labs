# Logic of the path_tracker_listener node

This workspace implements a ROS 2 node, `path_tracker_listener`, designed to analyze AprilTag data (attached rispectively on a floor frame, a chargin station frame, and a vacuum cleaner frame. All the data are seen by a camera in which is atached its respective frame) from a ROS bag and calculate projected 2D paths

---

The goal is to determine the 2D path (XY) of an autonomous vacuum cleaner (VC) and a charging station (CS) relative to the Floor (`tag36h11:0`) and the path of the VC relative to the Charging Station (CS, `tag36h11:1`)

---



###  TF Listener & Buffer

The node uses a **`tf2_ros::Buffer`** as a local memory cache for storing incoming 3D transformations (the rototranslation matrix from the vc and cs w.r.t the camera frame).

A **`tf2_ros::TransformListener`** silently subscribes to the `/tf` and `/tf_static` topics to continuously feed the Buffer with data

###  Logic of the node

The `on_timer()` function requests VC to Floor and CS to Floor transformations through the `lookupTransform` function of the  **`tf2_ros::Buffer`**. Then save the x and y component of the new rototraslational matrices on a first csv vector.

The VC w.r.t CS path is calculated via the **subtraction** of the projected X and Y coordinates from this previous matrices, and then stored in a apposite second csv vector.

This ensures both the global and relative 2D plots are geometrically consistent.

###  Output
At the end the projected paths (global and relative) are saved into two separate CSV files, reading from the previous 2 csv vector, upon node shutdown (`Ctrl+C`) and plotted by a python script `plot_path.py`