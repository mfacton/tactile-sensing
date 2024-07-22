#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from tools.serial_manager import SerialManager

manager = SerialManager("probe-board", baud=921600)
rospy.init_node("stm_node", anonymous=True)
sensor_topic = rospy.Publisher("pressures", Float32MultiArray, queue_size=10)
temperature_topic = rospy.Publisher("temperatures", Float32MultiArray, queue_size=10)

cal_count = 20

pout_indexes = [0, 1, 2, 3, 4, 5, 6, 7]
tout_indexes = [0, 1, 2, 3, 4, 5, 6, 7]

pout_len = len(pout_indexes)
tout_len = len(tout_indexes)

pout_accum = [0 for _ in range(pout_len)]
pout_offset = [0 for _ in range(pout_len)]
tout_accum = [0 for _ in range(tout_len)]
tout_offset = [0 for _ in range(tout_len)]

def read_data():
    pressures = []
    temperatures = []
    
    data = manager.read_bytes(40)
    
    for s in range(8):
        start_index = s * 5
        pressures.append(
            data[start_index + 2] << 16 | data[start_index + 1] << 8 | data[start_index]            
        )
        temperatures.append(
            data[start_index + 4] << 8 | data[start_index + 3]
        )

    return pressures, temperatures


def read_select():
    pressures, temperatures = read_data()
    pout_data = []
    tout_data = []

    for i in range(pout_len):
        pout_index = pout_indexes[i]
        pout_data.append(pressures[pout_index] + pout_offset[i])

    for i in range(tout_len):
        tout_index = tout_indexes[i]
        tout_data.append(temperatures[tout_index] + tout_offset[i])
    
    return pout_data, tout_data


for ep in range(cal_count):
    pout_data, tout_data = read_select()
    
    for i in range(pout_len):
        pout_accum[i] += pout_data[i]
        
    for i in range(tout_len):
        tout_accum[i] += tout_data[i]

pout_avg = sum(pout_accum)
pout_avg /= cal_count * pout_len
tout_avg = sum(tout_accum)
tout_avg /= cal_count * tout_len

for i in range(pout_len):
    pchan_avg = pout_accum[i] / pout_cal_count
    pout_offset[i] = pout_avg - pchan_avg

for i in range(tout_len):
    tchan_avg = tout_accum[i] / tout_cal_count
    tout_offset[i] = tout_avg - tchan_avg

while True:
    pout_data, tout_data = read_select()
    
    for i in range(pout_len):
        pout_data[i] = (100 * pout_data[i]) / 4096 # convert to pascals
    
    for i in range(tout_len):
        tout_data[i] = tout_data[i] / 100

    p_packet = Float32MultiArray(data=np.array(pout_data))
    sensor_topic.publish(p_packet)
    t_packet = Float32MultiArray(data=np.array(tout_data))
    temperature_topic.publish(t_packet)

    # print(pout_data, tout_data)

