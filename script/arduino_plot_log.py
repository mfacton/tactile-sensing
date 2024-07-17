import struct
import sys
import time

from tools.logger import Logger
from tools.plotter import Plot
from tools.serial_manager import DeviceNotFound, SerialManager

NUM_ADC = 12

managers = []

device_num = 0
while True:
    try:
        manager = SerialManager("Arduino Micro", device_number=device_num, baud=57600)
        managers.append(manager)
        print(f"Device {device_num} log started")
    except DeviceNotFound:
        break
    device_num += 1

if device_num == 0:
    print("No devices found")
    sys.exit(1)

plotter = Plot(
    data_length=1,
    height_scale=1,
    pixel_shift=3,
    title=f"Arduino Micro {device_num}",
)
# logger = Logger("data/log1.csv")

while True:
    all_values = []
    data = True
    for manager in managers:
        data = manager.read_line()
        if len(data) == NUM_ADC * 2 + 2:
            all_values += struct.unpack(f"<{NUM_ADC}H", data[1 : NUM_ADC * 2 + 1])
            # print(all_values)
        else:
            data = False

    if data:
        all_values[0] = 512 - all_values[0] / 2
        plotter.push(all_values[0:1])
#        logger.push(all_values)
