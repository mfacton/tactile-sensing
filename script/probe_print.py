import time

from tools.serial_manager import SerialManager

manager = SerialManager("probe-board", baud=460800)
# manager = SerialManager("probe-eight", baud=460800)

sensors = 8

def read_data():
    pressures = []
    temperatures = []
    data = manager.read_bytes(sensors*5)
    # print(data)
    for s in range(sensors):
        start_index = s * 5
        pressures.append(
            data[start_index + 2] << 16 | data[start_index + 1] << 8 | data[start_index]
        )
        temperatures.append(data[start_index + 4] << 8 | data[start_index + 3])

    return pressures, temperatures


while True:
    pressures, temperatures = read_data()
    print("========")
    for s in range(sensors):
        print(f"Sensor {s:2d}: {pressures[s]/40960:8f}, {temperatures[s]/100:4f}")
    time.sleep(0.001)
