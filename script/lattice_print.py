from tools.serial_manager import SerialManager

manager = SerialManager("LatticeBoard1")


def read_data():
    pressures = []
    temperatures = []
    data = manager.read_bytes(225)
    for s in range(45):
        start_index = s * 5
        pressures.append(
            data[start_index + 2] << 16 | data[start_index + 1] << 8 | data[start_index]
        )
        temperatures.append(data[start_index + 4] << 8 | data[start_index + 3])

    return pressures, temperatures


while True:
    pressures, temperatures = read_data()
    print("========")
    for s in range(45):
        print(f"Sensor {s:2d}: {pressures[s]/4096:8f}, {temperatures[s]/100:4f}")
