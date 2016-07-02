# Calibration

For calibration, first record a rosbag with a preliminary, hand made estimate of the landmarks position. (You can do this by setting the _record_rosbag_ parameter to true in the _stargazer_demo_ launch file).

You can then start the calibration with:
~~~{.shell}
roslaunch stargazer_ros landmark_calibrator.launch
~~~
