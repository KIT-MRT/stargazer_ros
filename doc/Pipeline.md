# The Stargazer Pipeline
The library can be used independent of the ROS executables and holds all functionality, ready to be integrated into your framework. Everso, the ROS package comes with additional tools for debugging and visualization. In the ROS package, functionality is divided into three different executables, namely **LandmarkFinder**, **LandmarkLocalizer** and **LandmarkCalibrator**, which can be  run as nodes or nodelets. This makes use of the modular structure of ROS and allows for pipelining, a concept to increase the frequency at which data is available at the output despite latencies in the process chain.

For ease of use when fine tuning the system, all parameters can be set online using the *rqt_dynamic_reconfigure* package. Finally, additional tools for visualizing landmark and robot poses in *rviz* as well as for visualizing the reprojection errors are available.

Get started by running the stargazer demo launch file

~~~{.shell}
roslaunch stargazer_ros stargazer_demo.launch
~~~

This will start the following pipeline:

_/undistorted_image_ -> **LandmarkFinder** -> _/landmarks_seen_  -> **LandmarkLocalizer** -> _/camera_pose_

The result can be visualized with rviz.
