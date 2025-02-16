#!/usr/bin/env python3

# MARKER: IMPORTS
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import serial
from serial.serialutil import SerialException

# MARKER: CLASS_DEFINITION
class ArduinoSerialNode(Node):
    """
    ROS2 Node for communicating with Arduino via Serial
    """
    def __init__(self):
        """Initialize the node and serial communication"""
        super().__init__('arduino_serial')
        
        try:
            # MARKER: SERIAL_SETUP
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=57600,
                timeout=1.0
            )
            self.get_logger().info('Successfully connected to Arduino')
        except SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            raise

        # MARKER: PUBLISHER_SETUP
        self.publisher = self.create_publisher(
            msg_type=String,
            topic='sensor_data',
            qos_profile=10
        )
        
        # MARKER: TIMER_SETUP
        self.timer = self.create_timer(0.1, self.read_serial)
        self.get_logger().info('Node initialized successfully')

    def read_serial(self):
        """Read data from serial port and publish to ROS2 topic"""
        try:
            if self.serial_port.in_waiting:
                data = self.serial_port.readline().decode('utf-8').strip()
                msg = String()
                msg.data = data
                self.publisher.publish(msg)
                self.get_logger().debug(f'Published: {data}')
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {str(e)}')

# MARKER: MAIN_FUNCTION
def main(args=None):
    """Main function to initialize and run the node"""
    try:
        rclpy.init(args=args)
        node = ArduinoSerialNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        # Cleanup
        rclpy.shutdown()

if __name__ == '__main__':
    main()