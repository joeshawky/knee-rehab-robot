#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


import socket
import subprocess
import time
import threading
from knee_rehab_interfaces.msg import ModeStatus, AngleTimeStamped, BatteryStatusTimeStamped

from knee_rehab_py_pkg.utilities.knee_rehab_modes import KneeRehabModes
from knee_rehab_py_pkg.utilities.bluetooth import BluetoothServer, BluetoothDataManager

from rclpy.impl.rcutils_logger import RcutilsLogger

class BluetoothManagerNode(Node): 
    def __init__(self):
        super().__init__("bluetooth_server")
        self.init_parameters()
        self.init_variables()
        self.init_bluetooth_server()
        self.init_publishers()
        self.init_subscriptions()
        self.start_client_handler()        
        self.logger.info("Bluetooth server node has been initialised.")


    def init_parameters(self):
        self.declare_parameter("mode_publisher_timer", 1.0)
        self.declare_parameter("client_commands_send_timer", 1.0)


    def init_variables(self) -> None:

        self.logger: RcutilsLogger = self.get_logger()

        self.current_mode = KneeRehabModes.Modes.FREE_MODE
        self.current_mode_status = KneeRehabModes.Status.MODE_STOP

        self.mode_publisher_timer = self.get_parameter("mode_publisher_timer").value
        self.client_commands_send_timer = self.get_parameter("client_commands_send_timer").value


    def init_bluetooth_server(self):
        self.bluetooth_server = BluetoothServer(self.logger, self.handle_command, "knee rehab")
        self.data_manager = BluetoothDataManager()
        self.bluetooth_server.remove_paired_devices()

        scan_pair_thread = threading.Thread(target=self.bluetooth_server.scan_and_pair())
        scan_pair_thread.start()



    def init_publishers(self) -> None:
        self.mode_publisher = self.create_publisher(ModeStatus, "current_mode", 10)
        self.mode_publisher_timer = self.create_timer(self.mode_publisher_timer, self.publish_mode)


    def publish_mode(self) -> None:
        msg = ModeStatus()
        msg.mode = self.current_mode
        msg.is_active = self.current_mode_status
        self.mode_publisher.publish(msg)

    def set_mode(self, mode:KneeRehabModes.Modes, mode_status:KneeRehabModes.Status) -> None:
        self.current_mode = mode
        self.current_mode_status = mode_status


    def init_subscriptions(self) -> None:
        self.create_subscription(AngleTimeStamped, "current_angle", self.data_manager.callback_current_knee_angle, 10)
        self.create_subscription(AngleTimeStamped, "target_angle", self.data_manager.callback_target_knee_angle, 10)
        self.create_subscription(BatteryStatusTimeStamped, "battery_status", self.data_manager.callback_battery_status, 10)
        self.create_subscription(BatteryStatusTimeStamped, "motor_power", self.data_manager.callback_motor_power, 10)
        self.create_subscription(ModeStatus, "current_mode", self.data_manager.callback_current_mode, 10)
        
        self.create_timer(0.1, self.update_bluetooth_server)


    def update_bluetooth_server(self) -> None:
        msg = self.data_manager.get_data()
        self.bluetooth_server.update_data(msg)
        # self.logger.info(f"{msg}")
    

    def start_client_handler(self):
        threading.Thread(target=self.client_manager, daemon=True).start()
    

    def client_manager(self):
        while True:
            self.logger.info("Waiting for a client connection...")

            client_sock, _ = self.bluetooth_server.server_socket.accept()
            self.logger.info("Client connected.")
            self.bluetooth_server.client_connected = True

            recv_thread = threading.Thread(target=self.bluetooth_server.client_receive_worker, args=(client_sock,))
            send_thread = threading.Thread(target=self.bluetooth_server.client_send_worker, args=(client_sock, self.client_commands_send_timer))
            
            recv_thread.start()
            send_thread.start()

            recv_thread.join()
            send_thread.join()

            self.logger.info("Client handler restarted due to disconnection.")


    def handle_command(self, command: str):
        self.logger.info(f"Received command: {command}")

        if "set_mode=" in command:
            command = command.replace("set_mode=", "").split(':')
            
            mode = command[0]
            status = command[1]
            available_mode = mode in KneeRehabModes._mode_names.values() and status in KneeRehabModes._status_names.values()
            if available_mode:
                mode_id = KneeRehabModes.get_mode_id(mode)
                status_id = KneeRehabModes.get_status_id(status)
                self.set_mode(mode_id, status_id)
        #set_mode=FREE_MODE:MODE_START
        #set_mode:mode-active       
        # set_mode=LEARNING_MODE-MODE_STOP 
        #Command handling logic for setting mode.
        # self.set_mode()

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothManagerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    node.server_socket.close()

if __name__ == "__main__":
    main()