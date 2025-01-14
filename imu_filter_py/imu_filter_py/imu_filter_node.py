import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.signal import bilinear, lfilter
from collections import deque

class IMUFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')

        # Initialize a buffer for the moving average filter
        self.window_size = 300
        self.accel_buffers = {
            'x': deque(maxlen=self.window_size),
            'y': deque(maxlen=self.window_size),
            'z': deque(maxlen=self.window_size)
        }

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(
            Imu,
            '/imu/filtered',
            10
        )

    def imu_callback(self, msg):
        """Callback function to process incoming IMU data."""
        # Get the raw acceleration data
        raw_accel = {
            'x': msg.linear_acceleration.x,
            'y': msg.linear_acceleration.y,
            'z': msg.linear_acceleration.z
        }

        # Update buffers and calculate moving average
        filtered_accel = {}
        for axis in ['x', 'y', 'z']:
            self.accel_buffers[axis].append(raw_accel[axis])
            filtered_accel[axis] = sum(self.accel_buffers[axis]) / len(self.accel_buffers[axis])

        # Update and publish filtered IMU message
        msg.linear_acceleration.x = filtered_accel['x']
        msg.linear_acceleration.y = filtered_accel['y']
        msg.linear_acceleration.z = filtered_accel['z']
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
