#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from knee_rehab_interfaces.msg import BatteryStatusTimeStamped
from rclpy.impl.rcutils_logger import RcutilsLogger


class INA219Simulation(Node): 
    def __init__(self):
        super().__init__("ina219_simulation") 
        self.init_parameters()
        self.init_variables()
        self.init_publishers()
        self.get_logger().info("ina219 simulation node has been initialised.")


    def init_parameters(self):
        self.declare_parameter("ina219_publisher_timer", 1.0)
        self.declare_parameter("counter", 1.0)
        self.declare_parameter("min_value", 1.0)
        self.declare_parameter("max_value", 1.0)

    def init_variables(self):
        self.increasing: bool = True # Temporary variable
        self.logger: RcutilsLogger = self.get_logger()
        self.ina219_publisher_timer = self.get_parameter("ina219_publisher_timer").value
        self.counter = self.get_parameter("counter").value
        self.min_value = self.get_parameter("min_value").value
        self.max_value = self.get_parameter("max_value").value
        self.data: float = self.min_value


    def init_publishers(self):
        self.publisher = self.create_publisher(BatteryStatusTimeStamped, "ina219_data", 10)
        self.timer = self.create_timer(self.ina219_publisher_timer, self.publish_power_status)


    def publish_power_status(self):
        if (self.data == self.min_value):
            self.increasing = True
        elif (self.data == self.max_value):
            self.increasing = False
        
        if self.increasing:
            self.data += self.counter
        else:
            self.data -= self.counter
        
        msg = BatteryStatusTimeStamped()
        msg.voltage = self.data
        msg.current = self.data
        msg.time = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = INA219Simulation() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
