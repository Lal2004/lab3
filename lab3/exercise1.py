#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal
import threading

class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.get_logger().info("Circle driver node started")

    def drive_circle(self):
        twist = Twist()
        # Set linear and angular velocities for a circle
        twist.linear.x = 0.2  # m/s
        twist.angular.z = 0.5  # rad/s
        # Radius = linear.x / angular.z = 0.4m
        
        while rclpy.ok():
            self.publisher.publish(twist)
            self.rate.sleep()

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info("Stopping robot")

def main(args=None):
    rclpy.init(args=args)
    circle_driver = CircleDriver()
    
    # Handle CTRL+C shutdown
    def signal_handler(sig, frame):
        circle_driver.stop()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, signal_handler)
    
    # Run ROS in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(circle_driver,), daemon=True)
    thread.start()
    
    try:
        circle_driver.drive_circle()
    except KeyboardInterrupt:
        pass
    
    circle_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()