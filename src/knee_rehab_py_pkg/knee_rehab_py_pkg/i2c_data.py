#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from knee_rehab_interfaces.msg import BatteryStatusTimeStamped 
from knee_rehab_interfaces.msg import AngleTimeStamped
from knee_rehab_interfaces.msg import ForceTimeStamped

import smbus

from knee_rehab_py_pkg.utilities.i2c_devices import AS5600, INA219, ADS1115

class I2cDataNode(Node): 
    def __init__(self):
        super().__init__("i2c_data") 

        self.bus = smbus.SMBus(1)
        self.as5600 = AS5600(self.bus)
        self.ina219 = INA219(self.bus)
        self.ina219.configure()
        self.ads1115 = ADS1115(self.bus, pga=ADS1115.PGA_6_144V, data_rate=ADS1115.DR_16SPS)

        self.init_parameters()
        self.init_variables()
        self.init_publishers()
        self.logger.info("I2C data node has been initialised.")


    def init_parameters(self):
        self.declare_parameter("data_rate", 1.0)
        self.declare_parameter("sensor_to_power_update_ratio", 1.0)

    def init_variables(self):
        # self.data: float = 0.0
        # self.increasing: bool = True # Temporary variable
        self.logger: RcutilsLogger = self.get_logger()
        self.sensor_to_power_update_ratio = self.get_parameter("sensor_to_power_update_ratio").value
        self.counter = 0
        
        data_rate = self.get_parameter("data_rate").value
        self.create_timer(data_rate, self.publish_sensor_data)


    def init_publishers(self):
        self.as5600_publisher = self.create_publisher(AngleTimeStamped, "as5600", 10)
        self.ina219_publisher = self.create_publisher(BatteryStatusTimeStamped, "ina219", 10)
        self.ads1115_publisher = self.create_publisher(ForceTimeStamped, "ads1115", 10)


    def publish_motor_power(self, voltage_data, current_data):
        msg = BatteryStatusTimeStamped()
        msg.voltage = voltage_data #V
        msg.current = current_data #A
        msg.time = self.get_clock().now().to_msg()
        self.ina219_publisher.publish(msg)


    def publish_angle(self, angle_data):
        #Real angle must be read first then published!
        # self.publish_simulated_angle()
        msg = AngleTimeStamped()
        msg.angle = angle_data
        msg.time = self.get_clock().now().to_msg()
        self.as5600_publisher.publish(msg)


    def publish_force(self, force_data):
        msg = ForceTimeStamped()
        msg.force = force_data
        msg.time = self.get_clock().now().to_msg()
        self.ads1115_publisher.publish(msg)
    
        
    def publish_sensor_data(self):
        try:
            # AS5600 data
            angle = self.as5600.read_angle()
            # self.logger.info(f'AS5600 Angle: {angle:.2f}°, counter: {self.counter}')
            self.publish_angle(angle)


            # ADS1115 data
            # differential_force = self.ads1115.read_differential(ADS1115.MUX_DIFF_0_1)
            # self.get_logger().info(f'ADS1115 Differential: {differential_force:.4f} V')
            # self.publish_force(differential_force)

            if self.counter == self.sensor_to_power_update_ratio:
                # INA219 data
                bus_voltage = self.ina219.read_bus_voltage()
                current = self.ina219.read_current()
                temp_shunt = self.ina219.read_shunt_voltage()
                # self.logger.info(f'AS5600 Angle: {angle:.2f}°, counter: {self.counter}')

                self.logger.info(f'AS5600 Angle: {angle:.2f}°, counter: {self.counter}Voltage: {bus_voltage:.2f}v, temp_shunt: {temp_shunt:.2f}v, Current: {current}')

                self.publish_motor_power(bus_voltage, current)
                self.counter = 0
            
            self.counter += 1

        except Exception as e:
            self.get_logger().error(f'Error reading sensor data: {e}')



    def publish_simulated_angle(self):
        if (self.data == 0.0):
            self.increasing = True
        elif (self.data == 10.0):
            self.increasing = False
        
        if self.increasing:
            self.data += 0.25
        else:
            self.data -= 0.25
        
        msg = AngleTimeStamped()
        msg.angle = self.data
        msg.time = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = I2cDataNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
