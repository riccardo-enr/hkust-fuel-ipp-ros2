#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os
import signal
import subprocess

class ExplorationSupervisor(Node):
    def __init__(self):
        super().__init__('exploration_supervisor')
        
        self.declare_parameter('threshold', 80.0)
        self.threshold = self.get_parameter('threshold').value
        
        self.subscription = self.create_subscription(
            Float32,
            '/sdf_map/explored_volume',
            self.volume_callback,
            10)
        
        self.get_logger().info(f'Exploration Supervisor Started. Target: {self.threshold}%')

    def volume_callback(self, msg):
        current_pct = msg.data
        if current_pct >= self.threshold:
            self.get_logger().info(f'EXPLORATION TARGET REACHED! ({current_pct:.2f}% >= {self.threshold}%)')
            self.get_logger().info('Shutting down simulation via pkill...')
            
            # Kill the launch process and all children matching the launch command
            subprocess.run(["pkill", "-f", "ros2 launch plan_bringup fuel_pipeline.launch.py"])
            
            # Suicide to ensure this node stops too
            os.kill(os.getpid(), signal.SIGINT)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
