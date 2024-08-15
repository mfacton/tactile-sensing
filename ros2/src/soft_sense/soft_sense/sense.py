#!/usr/bin/env python3
import struct
import sys

import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def get_active_ports():
    all_ports = serial.tools.list_ports.comports()
    active_ports = []
    for port in all_ports:
        if port.description != "n/a":
            active_ports.append(port)
    return active_ports

class SenseNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("sense")

        #init publisher topics
        self.pressure_pub = self.create_publisher(Float32MultiArray, "/pressure", 10)
        self.temperature_pub = self.create_publisher(Float32MultiArray, "/temperature", 10)

        # init calibration variables
        self.poffsets = [0 for x in range(8)]
        self.toffsets = [0 for x in range(8)]

        self.ser = None

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
    
    def calibrate(self, measurements):
        self.get_logger().info("Starting calibration")
        
        paverages = [0 for x in range(8)]
        taverages = [0 for x in range(8)]

        for m in range(measurements):
            pressures, temperatures = self.read_data()
            for i in range(8):
                paverages[i] += pressures[i]
                taverages[i] += temperatures[i]
        
        pavgall = sum(paverages)/8/measurements
        tavgall = sum(taverages)/8/measurements

        for i in range(8):
            self.poffsets[i] = paverages[i]/measurements - pavgall
            self.toffsets[i] = taverages[i]/measurements - tavgall

        self.get_logger().info("Finished calibration")
        self.get_logger().info(f"Pressure Offsets: {str([int(p) for p in self.poffsets])}")
        self.get_logger().info(f"Temperature Offsets: {str([int(t) for t in self.toffsets])}")


    def run(self):
        self.get_logger().info("Starting publishing pressures and temperatures")
        while True:
            pressures, temperatures = self.read_data()
            
            pdata = Float32MultiArray()
            tdata = Float32MultiArray()

            pdata.data = pressures
            tdata.data = temperatures
            
            self.pressure_pub.publish(pdata)
            self.temperature_pub.publish(tdata)


def main(args=None):
    rclpy.init(args=args)
    node = SenseNode()
    node.calibrate(200)
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()