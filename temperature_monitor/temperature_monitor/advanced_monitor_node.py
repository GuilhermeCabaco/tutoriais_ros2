#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class AdvancedTemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__('advanced_monitor_node')

        # TODO: Declare parameters for warning and critical thresholds
        self.declare_parameter('warning_threshold', 25.0)   # default warning threshold
        self.declare_parameter('critical_threshold', 27.5)  # default critical threshold
        
        self.warning_threshold = self.get_parameter('warning_threshold').value
        self.critical_threshold = self.get_parameter('critical_threshold').value
        
        self.get_logger().info(
            f'Advanced Temperature Monitor started: warning_threshold={self.warning_threshold}°C, 'f'critical_threshold={self.critical_threshold}°C'
        )

        # TODO: Create a subscriber with appropriate QoS settings
        self.subscription = self.create_subscription(
            String,                     
            'enhanced_temperature',     
            self.temperature_callback,    
            10                          
        )

        # TODO: Initialise counters for different alert levels
        self.warning_count = 0
        self.critical_count = 0

        # TODO: Set up moving average window
        self.window_size = 5
        self.readings = []

    def temperature_callback(self, msg):
        # TODO: Parse JSON temperature data
        data = json.loads(msg.data)
        temperature = data.get('temperature', None)

        # TODO: Apply moving average filter
        self.readings.append(temperature)

        if len(self.readings) > self.window_size:
            self.readings.pop(0)

        avg_temp = sum(self.readings) / len(self.readings)

        
        # TODO: Check against thresholds and issue appropriate alerts
        if avg_temp > self.critical_threshold:
            self.critical_count += 1
            self.get_logger().error(
                f'CRITICAL ALERT: {avg_temp:.1f}°C exceeds critical threshold of {self.critical_threshold}°C'
            )
        elif avg_temp > self.warning_threshold:
            self.warning_count += 1
            self.get_logger().warn(
                f'WARNING: {avg_temp:.1f}°C exceeds warning threshold of {self.warning_threshold}°C'
            )
        else:
            self.get_logger().info(f'Temperature {avg_temp:.1f}°C is within normal range.')
        
        # TODO: Update alert counters
        self.get_logger().info(
            f'Alert counts - Warnings: {self.warning_count}, Criticals: {self.critical_count}'
        )

        pass

def main(args=None):
    # TODO: Initialise node and handle lifecycle
    rclpy.init(args=args)
    node = AdvancedTemperatureMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    pass

if __name__ == '__main__':
    main()