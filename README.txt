This package has been tested on ROS2 Humble and has two functionalities, filtering IMU acceleration data and getting IMU trajectory
from the IMU acceleration data to compare with LiDAR odometry taken from the topics provided by LeGO-LOAM-SR. 

The filtering is done with a moving average filter with a window size of 300. To run the IMU data filtering node
run this command:

    ros2 run imu_filter_py filter

The filtered data will be published to a topic named /imu/filtered.

To check if the filtering is done correctly the odometry_check node plots the trajectory calculated from IMU by
double integrating the acceleration data which gives us position and LiDAR odometry created by the LeGO-LOAM-SR package.
To use the odometry_check node first you need to run the filtering node(if you want to check the raw IMU data
change the topic that the filtering node subscribes to to /imu/data), then launch LeGO-LOAM-SR with this command:

    ros2 launch lego_loam_sr run.launch.py

If you have recorded data in a rosbag file start playing it and then run the odometry_check node with this command:

    ros2 run imu_filter_py odom_compare

Let the .db3 file play for how ever long you need it to and then you can press ctrl + c in the terminal where odometry_check
is running, a plot with two graphs will come up showing where in one of them there will be the IMU trajectory and in the
other one will be LiDAR odometry.
