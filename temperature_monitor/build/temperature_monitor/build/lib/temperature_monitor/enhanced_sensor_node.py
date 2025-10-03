#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json

class EnhancedTemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('enhanced_temperature_sensor')

        # TODO: Declare parameters for temperature range and publishing rate
        self.declare_parameter('min_temp', 15.0)      # default minimum temperature
        self.declare_parameter('max_temp', 35.0)      # default maximum temperature
        self.declare_parameter('publish_rate', 3.0)   # default rate in seconds
        self.declare_parameter('location', 'DEEC')     # default location

        # Get parameter values
        self.min_temp = self.get_parameter('min_temp').value
        self.max_temp = self.get_parameter('max_temp').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.location = self.get_parameter('location').value

        # TODO: Create a publisher with appropriate QoS settings
        self.publisher = self.create_publisher(
            String,                  
            'enhanced_temperature',  
            10                       # QoS
        )

        # TODO: Create a timer based on the configured rate
        self.timer = self.create_timer(
            self.publish_rate,
            self.timer_callback
        )

        # TODO: Initialise sensor location and other metadata
        self.get_logger().info(
            f"Enhanced Temperature Sensor started: range=({self.min_temp}, {self.max_temp}), "
            f"rate={self.publish_rate} Hz, location='{self.location}'"
        )

    def timer_callback(self):
        # TODO: Generate temperature reading with metadata
        temperature = round(random.uniform(self.min_temp, self.max_temp), 1
        )

        reading = {
            'temperature': temperature,
            'unit': 'ºC',
            'location': self.location,
            'timestamp': self.get_clock().now().to_msg().sec
        }

        msg = String()
        msg.data = json.dumps(reading)  

        # TODO: Publish reading as a JSON string
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing reading: {msg.data}')

        pass

def main(args=None):
    # TODO: Initialise node and handle lifecycle
    rclpy.init(args=args)
    node = EnhancedTemperatureSensorNode()

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