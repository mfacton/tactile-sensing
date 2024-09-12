#!/usr/bin/env python3
import sys
import time

import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from soft_msgs.srv import Calibrate
from std_msgs.msg import Float32MultiArray


def get_active_ports():
    all_ports = serial.tools.list_ports.comports()
    active_ports = []
    for port in all_ports:
        if port.description != "n/a":
            active_ports.append(port)
    return active_ports

class SenseNode(Node):
    def __init__(self, cal_meas=200):
        # This is the actual name of the node that will be in ros
        super().__init__("sense")

        #init publisher topics
        self.pressure_pub = self.create_publisher(Float32MultiArray, "/pressure", 10)
        self.temperature_pub = self.create_publisher(Float32MultiArray, "/temperature", 10)
        self.srv = self.create_service(Calibrate, 'calibrate', self.cal_service)

        # init calibration variables
        self.poffsets = [0 for x in range(8)]
        self.toffsets = [0 for x in range(8)]

        self.ser = None
        self.cal = cal_meas

        ports = get_active_ports()
        for port in ports:
            self.get_logger().info(f"Discovered: {port.description} at {port.device}")

        for port in ports:
            if port.description == "probe-board":
                self.get_logger().info(f"Connecting to {port.description} at {port.device}")
                self.ser = serial.Serial(port.device, 460800)
                break
        
        if not self.ser:
            self.get_logger().error("Probe board not found")
            sys.exit(1)
        
        self.get_logger().info("Started publishing pressures and temperatures")

    def read_data(self):
        data = self.ser.read(40)
        pressures = []
        temperatures = []
        for i in range(8):
            start_index = i * 5
            pressures.append(
                (data[start_index + 2] << 16 | data[start_index + 1] << 8 | data[start_index])/40.96-self.poffsets[i]
            )
            temperatures.append((data[start_index + 4] << 8 | data[start_index + 3])/100 - self.toffsets[i])

        return pressures, temperatures
    
    def cal_service(self, request, response):
        self.cal = request.measurements
        self.get_logger().info(f"Received calibration request len: {self.cal}")

        return response

    def calibrate(self):
        if self.cal == 0:
            return
        
        paverages = [0 for x in range(8)]
        taverages = [0 for x in range(8)]

        self.poffsets = [0 for x in range(8)]
        self.toffsets = [0 for x in range(8)]

        for m in range(self.cal):
            pressures, temperatures = self.read_data()
            for i in range(8):
                paverages[i] += pressures[i]
                taverages[i] += temperatures[i]
        
        pavgall = sum(paverages)/8/self.cal
        tavgall = sum(taverages)/8/self.cal

        for i in range(8):
            self.poffsets[i] = paverages[i]/self.cal - pavgall
            self.toffsets[i] = taverages[i]/self.cal - tavgall

        self.get_logger().info(f"Pressure Offsets: {str([int(p) for p in self.poffsets])}")
        self.get_logger().info(f"Temperature Offsets: {str([int(t) for t in self.toffsets])}")
        self.cal = 0


    def update(self):
        self.calibrate()
        pressures, temperatures = self.read_data()
        
        pdata = Float32MultiArray()
        tdata = Float32MultiArray()

        pdata.data = pressures
        tdata.data = temperatures
        
        self.pressure_pub.publish(pdata)
        self.temperature_pub.publish(tdata)


def main(args=None):
    rclpy.init(args=args)
    node = SenseNode(200)
    while True:
        node.update()
        rclpy.spin_once(node, timeout_sec=0.001)
    rclpy.shutdown()

if __name__ == '__main__':
    main()