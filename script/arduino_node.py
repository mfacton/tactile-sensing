#!/usr/bin/env python
import math
import struct

import numpy as np

# import rospy
from std_msgs.msg import Float32MultiArray

from tools.serial_manager import SerialManager

ADC_RESOLUTION = 10
FULL_SCALE_ADC_V = 5

CAL_COUNT = 20

QUEUE_SIZE = 10

pout_indexes = [0, 1, 2, 3, 6, 7, 9, 10, 11]
pout_len = len(pout_indexes)
pout_accum = [0 for _ in range(pout_len)]
pout_offset = [0 for _ in range(pout_len)]


manager = SerialManager("Arduino Micro", device_number=0)
rospy.init_node("arduino_node", anonymous=True)
pub = rospy.Publisher("arduino_sensor", Float32MultiArray, queue_size=QUEUE_SIZE)


def read_data():
    data = manager.read_line()
    return struct.unpack(f"<12H", data[1:25])


def read_select():
    pressures = read_data()
    pout_data = []

    for idx in range(pout_len):
        pout_index = pout_indexes[idx]
        pout_data.append(pressures[pout_index] + pout_offset[idx])

    return pout_data


def adc_to_float(val):
    return val / math.pow(2, ADC_RESOLUTION) * FULL_SCALE_ADC_V


# flush a few reads
read_data()
print(read_data())

# calibrate
for ep in range(CAL_COUNT):
    pout_data = read_select()
    for idx in range(pout_len):
        pout_accum[idx] += pout_data[idx]

pout_avg = sum(pout_accum)
pout_avg /= CAL_COUNT * pout_len

for idx in range(pout_len):
    chan_avg = pout_accum[idx] / CAL_COUNT
    pout_offset[idx] = pout_avg - chan_avg

# start publishing
while True:
    pout_data = read_select()
    for pidx in range(pout_len):
        pout_data[pidx] = adc_to_float(pout_data[pidx])

    # leave print statment as it leaves enough delay to no cause issues
    print(pout_data)
    packet = Float32MultiArray(data=pout_data)
    pub.publish(packet)
