# IMU Linear Acceleration Data Filtering and Testing

This ROS2 package has been developed and tested on ROS2 Humble. It provides two key functionalities:

1. **Filtering IMU Acceleration Data**: Applies a moving average filter to smooth the IMU acceleration data.
2. **Comparing IMU and LiDAR Trajectories**: Computes and visualizes the trajectory derived from filtered IMU data alongside LiDAR odometry from the LeGO-LOAM-SR package.

---

## Filtering IMU Data

The filtering process uses a moving average filter with a window size of 300. To run the IMU filtering node, use the following command:

```bash
ros2 run imu_filter_py filter
```

The filtered IMU data is published to the `/imu/filtered` topic.

### Important Notes:

- By default, the node subscribes to the `/imu/data` topic. If you want to test raw IMU data, ensure that this subscription remains unchanged.

---

## Visualizing and Comparing IMU and LiDAR Odometry

The `odometry_check` node calculates the IMU trajectory by double integrating the filtered acceleration data (to obtain position). It then compares this trajectory to LiDAR odometry from the LeGO-LOAM-SR package.

### Steps to Use the `odometry_check` Node:

1. **Run the IMU Filtering Node**:
   Ensure the filtering node is running if you want to use filtered IMU data. If you prefer to test with raw IMU data, modify the subscription topic of the filtering node to `/imu/data`.

2. **Launch LeGO-LOAM-SR**:
   Start the LeGO-LOAM-SR package using the following command:

   ```bash
   ros2 launch lego_loam_sr run.launch.py
   ```

3. **Play Recorded Data (if applicable):**
   If you have recorded data in a `.db3` ROS bag file, play it using:

   ```bash
   ros2 bag play <your_rosbag_file>.db3
   ```

4. **Run the ****`odometry_check`**** Node**:
   Execute the following command:

   ```bash
   ros2 run imu_filter_py odom_compare
   ```

5. **View the Plot**:

   - Allow the `.db3` file to play for as long as necessary.
   - Stop the `odometry_check` node by pressing `Ctrl + C` in the terminal.
   - A plot with two graphs will appear:
     - **Graph 1**: IMU-derived trajectory.
     - **Graph 2**: LiDAR odometry from LeGO-LOAM-SR.

---

This configuration provides a robust framework for evaluating the performance of the IMU filtering process. By comparing the IMU-derived trajectory against the LiDAR-based odometry, you can gain valuable insights into the accuracy and reliability of the system.

