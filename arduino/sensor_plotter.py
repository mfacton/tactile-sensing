import sys
sys.path.append('../tactile-sensing')

import struct
import time
from tools.serial_manager import SerialManager, DeviceNotFound
from tools.plotter import Plot

NUM_ADC = 9

managers = []

device_num = 0
while True:
    try:
        manager = SerialManager("Arduino Micro", device_number=device_num)
        managers.append(manager)
        print(f"Device {device_num} log started")
    except DeviceNotFound:
        break
    device_num += 1

plotter = Plot(data_length=NUM_ADC*len(managers), height_scale=0.5, pixel_shift=3, title="Device Plot")

while True:
    all_values = []
    for manager in managers:
        data = manager.read_bytes(18)
        all_values += struct.unpack('<9H', data)

    time.sleep(0.001)
    plotter.push(all_values)
