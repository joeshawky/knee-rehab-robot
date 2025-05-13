#!/usr/bin/env python3

from knee_rehab_interfaces.msg import AngleTimeStamped, BatteryStatusTimeStamped, ModeStatus
from knee_rehab_py_pkg.utilities.knee_rehab_modes import KneeRehabModes
from rclpy.impl.rcutils_logger import RcutilsLogger
import socket
import time
import pexpect
import re


class BluetoothDataManager: # MODIFY NAME
    def __init__(self):
        self.init_variables()   


    def get_data(self)->str:
        mode_name = KneeRehabModes.get_mode_name(self.current_mode_name)
        mode_status = KneeRehabModes.get_status_name(self.current_mode_status)
        msg = f"current_angle={self.current_knee_angle}:"\
              f"target_angle={self.target_knee_angle},{self.target_knee_angle_seconds}S:"\
              f"battery_status={round(self.battery_status_voltage, 5)}V,{round(self.battery_status_current, 5)}A:"\
              f"motor_power={round(self.motorpower_voltage, 5)}V,{round(self.motorpower_current, 5)}A:"\
              f"current_mode={mode_name} {mode_status}"

        return msg
         
    def init_variables(self):
        self.current_knee_angle = 0
        self.target_knee_angle = 0
        self.target_knee_angle_seconds = 0
        self.battery_status_voltage = 0
        self.battery_status_current = 0
        self.motorpower_voltage = 0
        self.motorpower_current = 0
        self.current_mode_name = 0
        self.current_mode_status = 0


    def callback_current_knee_angle(self, msg:AngleTimeStamped):
        self.current_knee_angle = msg.angle
        
    
    def callback_target_knee_angle(self, msg:AngleTimeStamped):
        self.target_knee_angle = msg.angle
        self.target_knee_angle_seconds = msg.time.sec

    def callback_battery_status(self, msg:BatteryStatusTimeStamped):
        self.battery_status_voltage = msg.voltage
        self.battery_status_current = msg.current

    def callback_motor_power(self, msg:BatteryStatusTimeStamped):
        self.motorpower_voltage = msg.voltage
        self.motorpower_current = msg.current

    def callback_current_mode(self, msg:ModeStatus):
        self.current_mode_name = msg.mode 
        self.current_mode_status = msg.is_active 


class BluetoothServer:
    def __init__(self, logger:RcutilsLogger, command_callback, client_mac_address:str):
        self.init_variables(logger, command_callback, client_mac_address)
        

    def init_variables(self, logger:RcutilsLogger, command_callback, client_mac_address:str):
        self.logger: RcutilsLogger = logger
        self.server_socket: socket.socket = self.initialize_tcp_socket()
        self.client_connected: bool = False
        self.latest_data: str = ""
        self.command_callback = command_callback
        self.current_ip: str = self.get_ip()
        self.pexpect: pexpect.spawn = pexpect.spawn('bluetoothctl', encoding='utf-8')
        self.client_mac_address = client_mac_address


    def initialize_tcp_socket(self) -> socket.socket:
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.bind(("172.18.227.18", 55556))
        server_sock.listen(1)
        self.logger.info(f"Listening on tcp socket port {server_sock.getsockname()[1]}")
        return server_sock

    
    def initialize_bluetooth_socket(self) -> socket.socket:
        server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server_sock.bind(("00:1a:7d:da:71:13", 4))
        server_sock.listen(1)
        self.logger.info(f"Listening on bluetooth socket port {server_sock.getsockname()[1]}")
        return server_sock


    def get_ip(self) -> str:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as temp_socket:
            temp_socket.connect(('10.255.255.255', 1))            
            return temp_socket.getsockname()[0]

        
    def client_receive_worker(self, client_sock:socket.socket) -> None:
        try:
            while self.client_connected:
                data:bytes = client_sock.recv(1024)
                if not data:
                    break
                self.command_callback(data.decode())
        except (ConnectionResetError, BrokenPipeError):
            self.logger.info("Client disconnected.")
            self.client_connected = False
        finally:
            client_sock.close()


    def client_send_worker(self, client_sock:socket.socket, delay_seconds:float):
        try:
            while self.client_connected:
                client_sock.sendall(self.latest_data.encode())
                # self.logger.info(f"Sent to client: {self.latest_data.encode()}")
                time.sleep(delay_seconds)
                
        except (ConnectionResetError, BrokenPipeError, OSError):
            self.logger.info("Client disconnected during send.")
            self.client_connected = False
        finally:
            client_sock.close()

    def update_data(self, data:str):
        self.latest_data = f"ip={self.current_ip}:{data}"
    

    # Pairing and Device Management
    def remove_paired_devices(self):
        self.pexpect.sendline("paired-devices")
        try:
            self.pexpect.expect("Device [0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}", timeout=5)
            deviceRows = self.pexpect.after.split('\n')[1:]
            for deviceRow in deviceRows:
                macAddress = deviceRow.split(' ')[1]
                self.pexpect.sendline(f"remove {macAddress}")
                self.pexpect.expect("Device has been removed")
                self.logger.info(f"Removed paired device: {macAddress}")
        except pexpect.TIMEOUT:
            self.logger.info("No paired devices found or timeout reached.")
        except pexpect.EOF:
            self.logger.info("EOF reached in remove_paired_devices")


    def init_pexpect(self):
        self.pexpect.sendline("pairable on")
        self.pexpect.sendline("agent NoInputNoOutput")


    def scan_and_pair(self):
        self.init_pexpect()
        while True:
            self.pexpect.sendline("scan on")
            try:
                # Regex to match device name
                self.pexpect.expect(r"NEW\sDevice\s([0-9A-F]{2}\:[0-9A-F]{2}\:[0-9A-F]{2}\:[0-9A-F]{2}\:[0-9A-F]{2}\:[0-9A-F]{2})\s" + re.escape(self.client_mac_address), timeout=20)
                mac_address = self.pexpect.match.group(1)
                self.pair_device(self.pexpect, mac_address)
            except pexpect.TIMEOUT:
                print(f"Device {self.client_mac_address} not found in scan.")
            except pexpect.EOF:
                print("EOF reached in scan_and_pair.")
                
    def pair_device(self, p, mac_address):
        p.sendline(f"pair {mac_address}")
        try:
            p.expect("Pairing successful", timeout=25)
            print(f"Successfully paired with device {mac_address}")
        except pexpect.TIMEOUT:
            print(f"Timeout reached while pairing with {mac_address}")
        except pexpect.EOF:
            print("EOF reached during pairing.")
