#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class TemperatureLoggerNode(Node):
    def __init__(self):
        super().__init__('temperature_logger')

        # TODO: Declare parameters for log file path and logging frequency
        self.declare_parameter('log_file_path', 'temperature_monitor/temperature_monitor/temperature_log.txt') 
        self.declare_parameter('log_frequency', 1)                    

        self.log_file_path = self.get_parameter('log_file_path').value
        self.log_frequency = self.get_parameter('log_frequency').value

        # TODO: Create a subscriber with appropriate QoS settings
        self.subscription = self.create_subscription(
            String,                     
            'enhanced_temperature',     
            self.logging_callback,    
            10                          
        )

        # TODO: Initialise logging counter and open log file
        self.log_counter = 0
        
        self.log_file = open(self.log_file_path, 'a')

        try:
            self.log_file = open(self.log_file_path, 'a')  # append mode
            self.get_logger().info(
                f"Logging to {self.log_file_path}, every {self.log_frequency} reading(s)."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open log file: {e}")
            raise

    def logging_callback(self, msg):
        # TODO: Parse temperature data
        data = json.loads(msg.data)
        temperature = data.get('temperature', None)
        timestamp = data.get('timestamp', None)
        location = data.get('location', 'Unknown')

        if temperature is None or timestamp is None:
            self.get_logger().warn("Received incomplete temperature data.")
            return

        # TODO: Check if this reading should be logged (based on frequency)
        self.log_counter += 1

        # TODO: Write to log file with timestamp
        if self.log_counter % self.log_frequency == 0:
            timestamp = datetime.now().isoformat(timespec='seconds')
            log_entry = f"{timestamp}, {temperature:.2f} °C, location={location}\n"
            self.log_file.write(log_entry)
            self.log_file.flush()  # make sure it’s written immediately

            self.get_logger().info(f"Logged: {log_entry.strip()}")

        pass

def main(args=None):
    # TODO: Initialise node and handle lifecycle
    # TODO: Ensure proper file cleanup on shutdown

    rclpy.init(args=args)
    node = TemperatureLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.log_file.close()  # Close the log file
        
if __name__ == '__main__':
    main()