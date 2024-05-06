import serial
import serial.tools.list_ports

class DeviceNotFound(Exception):
    """Cannot find device"""

class SerialManager:
    """Wrapper class for pyserial"""
    def __init__(self, device_name, baud=115200) -> None:
        self.baud = baud
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.description == device_name:
                self.device = port.device
                self.ser = serial.Serial(self.device, self.baud)
                return

        raise DeviceNotFound(f"{device_name} not found")

    def read_bytes(self, length):
        """Waits for length bytes recieved"""
        return self.ser.read(length)

    def read_line(self):
        """Read line from serial"""
        return self.ser.readline().decode("utf-8")
