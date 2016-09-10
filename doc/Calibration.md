# Calibration

For calibration, first record a rosbag with a preliminary, hand made estimate of the landmarks position. (You can do this by setting the _record_ to true in the _stargazer_nodelets_ launch file).

You can then start the calibration with:
~~~{.shell}
roslaunch stargazer_ros landmark_calibrator.launch
~~~
