# nodes/arduino_serial_node.py
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 57600)
        self.publisher = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting:
            data = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = data
            self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ArduinoSerialNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()