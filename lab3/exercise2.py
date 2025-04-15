#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal
import threading
import time

class SquareTracer(Node):
    def __init__(self):
        super().__init__('square_tracer')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.get_logger().info("Square tracer node started")

    def move_forward(self, duration):
        twist = Twist()
        twist.linear.x = 0.2  # m/s
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            self.rate.sleep()

    def turn(self, duration):
        twist = Twist()
        twist.angular.z = 0.5  # rad/s (90° in ~3.14s)
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            self.rate.sleep()

    def trace_square(self):
        side_duration = 2.0  # seconds for each side
        turn_duration = 3.14  # seconds for 90° turn
        
        while rclpy.ok():
            for _ in range(4):
                self.move_forward(side_duration)
                self.turn(turn_duration)

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info("Stopping robot")

def main(args=None):
    rclpy.init(args=args)
    square_tracer = SquareTracer()
    
    # Handle CTRL+C shutdown
    def signal_handler(sig, frame):
        square_tracer.stop()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, signal_handler)
    
    # Run ROS in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(square_tracer,), daemon=True)
    thread.start()
    
    try:
        square_tracer.trace_square()
    except KeyboardInterrupt:
        pass
    
    square_tracer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()