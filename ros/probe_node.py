#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from tools.serial_manager import SerialManager

manager = SerialManager("probe-board", baud=921600)
rospy.init_node("arduino_node", anonymous=True)
pub = rospy.Publisher("arduino_sensor", Float32MultiArray, queue_size=10)

pout_cal_count = 20

pout_indexes = [0, 1, 2, 3, 4, 5, 6, 7]

pout_len = len(pout_indexes)
pout_accum = [0 for _ in range(pout_len)]
pout_offset = [0 for _ in range(pout_len)]


def read_data():
    pressures = []
    data = manager.read_bytes(40)
    for s in range(8):
        start_index = s * 5
        pressures.append(
            data[start_index + 2] << 16 | data[start_index + 1] << 8 | data[start_index]
        )

    return pressures


def read_select():
    pressures = read_data()
    pout_data = []

    for idx in range(pout_len):
        pout_index = pout_indexes[idx]
        pout_data.append(pressures[pout_index] + pout_offset[idx])

    return pout_data


for ep in range(pout_cal_count):
    pout_data = read_select()
    for idx in range(pout_len):
        pout_accum[idx] += pout_data[idx]

pout_avg = sum(pout_accum)
pout_avg /= pout_cal_count * pout_len

for idx in range(pout_len):
    chan_avg = pout_accum[idx] / pout_cal_count
    pout_offset[idx] = pout_avg - chan_avg

while True:
    pout_data = read_select()
    for pidx in range(pout_len):
        pout_data[pidx] = (100 * pout_data[pidx]) / 4096

    packet = Float32MultiArray(data=np.array(pout_data))
    pub.publish(packet)
