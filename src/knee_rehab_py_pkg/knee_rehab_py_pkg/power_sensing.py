#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from knee_rehab_interfaces.msg import BatteryStatusTimeStamped as bs
from rclpy.impl.rcutils_logger import RcutilsLogger

class PowerSensingNode(Node): 
    def __init__(self):
        super().__init__("power_sensing") 
        self.init_parameters()
        self.init_variables()
        self.init_publishers()
        self.get_logger().info("Power sensing node has been initialised.")



    def init_parameters(self):
        self.declare_parameter("motor_power_publisher_timer", 1.0)


    def init_variables(self):
        self.logger: RcutilsLogger = self.get_logger()
        self.motor_power_publisher_timer: float = self.get_parameter("motor_power_publisher_timer").value

    
    def init_publishers(self):
        self.motorPowerPublisher = self.create_publisher(bs, "motor_power", 10)
        self.create_timer(self.motor_power_publisher_timer, self.publishMotorPower)


    def publishMotorPower(self):
        msg = bs()
        msg.voltage = 12.28 #V
        msg.current = 1.625 #A
        msg.time = self.get_clock().now().to_msg()
        self.motorPowerPublisher.publish(msg)
    


def main(args=None):
    rclpy.init(args=args)
    node = PowerSensingNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
