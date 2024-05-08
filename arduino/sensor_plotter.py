import sys
sys.path.append('../tactile-sensing')

import struct
from tools.serial_manager import SerialManager
from tools.plotter import Plot

NUM_ADC = 9

manager1 = SerialManager("Arduino Micro", device_number=1)
plotter = Plot(data_length=NUM_ADC, height_scale=0.5, pixel_shift=1)

while True:
    data = manager1.read_bytes(18)
    values = struct.unpack('<9H', data)
    plotter.push(values)
