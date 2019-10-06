# bebop_fly
ROS pacakage for tracking and landing on aruco marker using bebop

You should have the following ros packages installed:

1. cv_camera
2. image_proc
3. aruco_ros

In addition you should have calibrated your monocular camera using the camera_calibration ROS package

Then launch the land.launch file to run the nodes and detect aruco marker

If you are connected to bebop and running the bebop_node of bebop_driver package then you can run the lander.py node script to land on the aruco marker
