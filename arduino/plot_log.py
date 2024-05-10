import sys
sys.path.append('..')

import struct
import time
from tools.serial_manager import SerialManager, DeviceNotFound
from tools.plotter import Plot
from tools.logger import Logger

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

if not device_num:
    print("No devices found")
    sys.exit(1)

#plotter = Plot(data_length=NUM_ADC*len(managers), height_scale=0.5, pixel_shift=3, title="Device Plot")
#logger = Logger("data/log1.csv")
#logger.open()

while True:
    all_values = []
    data = True
    for manager in managers:
        data = manager.read_line()
        if len(data) == NUM_ADC*2+2:
            all_values += struct.unpack(f'<{NUM_ADC}H', data[1:NUM_ADC*2+1])
        else:
            data = False

    if data:
        time.sleep(0.001)
#        plotter.push(all_values)
#        logger.push(all_values)
