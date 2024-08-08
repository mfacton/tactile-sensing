import sys

sys.path.append("..")
import logging
import math
import time

from rtde import rtde, rtde_config

# ROBOT_HOST = "192.168.56.101" # simulation
ROBOT_HOST = "192.168.1.101" # real robot
ROBOT_PORT = 30004
config_filename = "config.xml"

logging.basicConfig(level=logging.INFO)
logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# Using a 10 Hz watchdog
watchdog.input_int_register_0 = 0

# start data synchronization
if not con.send_start():
    sys.exit()

def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp

def compare_positions(pos1, pos2):
    position_tolerance = 0.05
    rotation_tolerance = 0.001
    for i in range(3):
        if abs(pos1[i]-pos2[i]) > position_tolerance:
            return False
    
    for i in range(3, 6):
        if abs(pos1[i]-pos2[i]) > rotation_tolerance:
            return False
    
    return True

# Setpoints to move the robot to
center = [-0.44, 0.1, 0.2, math.pi, 0, 0]
radius = 0.2
height = 0.1
angle = 0
factor = 12

rad_sec = 0.4

position = None

def calculate_position():
    return [center[0]+radius*math.sin(angle), center[1]+radius*math.cos(angle), center[2]+height*(1+math.sin(angle*factor))/2, math.pi, 0, 0]


last_time = time.time()
homed = False
while True:
    # receive the current state
    state = con.receive()
    if state is None:
        print("state not received, exiting")
        break

    # print(state.actual_TCP_force)
    # for n in state.actual_TCP_force:
    #     print(f"{n:10.6f}")
    # print()

    position = calculate_position()
    list_to_setp(setp, position)
    con.send(setp)

    if not homed and compare_positions(position, state.actual_TCP_pose):
        homed = True
        last_time = time.time()
        print("homed")

    if homed and not compare_positions(position, state.actual_TCP_pose):
        print("off position execution, exiting")
        # break

    if homed:
        current_time = time.time()
        delta_time = current_time-last_time
        last_time = current_time
        angle += delta_time*rad_sec

    # do something...
    # if move_completed and state.output_int_register_0 == 1:
    #     move_completed = False
    #     new_setp = setp1 if setp_to_list(setp) == setp2 else setp2
    #     list_to_setp(setp, new_setp)
    #     print("New pose = " + str(new_setp))
    #     # send new setpoint
    #     con.send(setp)
    #     watchdog.input_int_register_0 = 1
    # elif not move_completed and state.output_int_register_0 == 0:
    #     print("Move to confirmed pose = " + str(state.target_q))
    #     move_completed = True
    #     watchdog.input_int_register_0 = 0

    # kick watchdog
    con.send(watchdog)

con.send_pause()

con.disconnect()
