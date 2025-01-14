import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Initialize subscriptions
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data', # Topic for IMU data
            self.imu_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init', # Topic for LiDAR odometry
            self.lidar_callback,
            10
        )

        # Data storage
        self.imu_data = []  # (angular velocity, linear acceleration, timestamp)
        self.lidar_positions = []  # (x, y, timestamp)
        self.start_time = None
        
        self.get_logger().info('TrajectoryPlotter node initialized.')

    def imu_callback(self, msg):
        # Extract IMU data (angular velocity, linear acceleration)
        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Set start time if not already set
        if self.start_time is None:
            self.start_time = timestamp

        # Normalize time
        relative_time = timestamp - self.start_time

        self.imu_data.append((angular_velocity, linear_acceleration, relative_time))

    def lidar_callback(self, msg):
        # Extract LiDAR odometry position
        position = msg.pose.pose.position
        x, y = position.x, position.y

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Normalize time
        if self.start_time is None:
            self.start_time = timestamp

        relative_time = timestamp - self.start_time

        self.lidar_positions.append((x, y, relative_time))

    def calculate_imu_trajectory(self):
        # Initializations
        angular_velocity_cumulative = np.array([0.0, 0.0, 0.0])
        position = np.array([0.0, 0.0, 0.0])

        trajectory = []
        dt = 1/500  # IMU sampling interval

        for angular_velocity, linear_acceleration, _ in self.imu_data:
            # Integrate angular velocity to estimate orientation
            angular_velocity_cumulative += angular_velocity * dt

            quaternion = R.from_euler('xyz', angular_velocity_cumulative).as_quat()
            quaternion /= np.linalg.norm(quaternion)  # Normalize quaternion

            rotation_matrix = R.from_quat(quaternion).as_matrix()

            # Transform acceleration to global frame
            acceleration_transformed = np.dot(rotation_matrix, linear_acceleration)

            # Integrate to get position (assuming velocity starts at 0)
            position += acceleration_transformed * (dt ** 2)  # x = 0.5 * a * t^2
            trajectory.append(position[:2].copy())  # Append x, y only

        return np.array(trajectory)

    def plot_trajectories(self):
        imu_trajectory = self.calculate_imu_trajectory()
        lidar_positions = np.array([[x, y, t] for x, y, t in self.lidar_positions])

        """
        imu_timestamps = [data[2] for data in self.imu_data]
        imu_accel_x = [data[1][0] for data in self.imu_data]
        imu_accel_y = [data[1][1] for data in self.imu_data]
        imu_accel_z = [data[1][2] for data in self.imu_data]
        """

        # Plot IMU and LiDAR trajectories
        plt.figure(figsize=(10, 12))

        """
        plt.subplot(3, 1, 1)
        plt.plot(imu_timestamps, imu_accel_x, label='Accel X')
        plt.plot(imu_timestamps, imu_accel_y, label='Accel Y')
        plt.plot(imu_timestamps, imu_accel_z, label='Accel Z')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s2)')
        plt.title('IMU Acceleration')
        plt.legend()
        plt.grid()
        plt.axis('equal')
        """

        plt.subplot(2, 1, 1)
        plt.plot(imu_trajectory[:, 0], imu_trajectory[:, 1], label='IMU Trajectory')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('IMU trajectory')
        plt.legend()
        plt.grid()
        plt.axis('equal')

        plt.subplot(2, 1, 2)
        plt.plot(lidar_positions[:, 0], lidar_positions[:, 1], label='LiDAR Odometry')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('LiDAR odometry')
        plt.legend()
        plt.grid()
        
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    trajectory_plotter = TrajectoryPlotter()

    try:
        rclpy.spin(trajectory_plotter)
    except KeyboardInterrupt:
        trajectory_plotter.get_logger().info('Stopping TrajectoryPlotter...')
        trajectory_plotter.plot_trajectories()
    finally:
        trajectory_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()