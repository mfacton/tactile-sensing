from tools.plotter import Plot, PlotColors
from tools.serial_manager import SerialManager

manager = SerialManager("lattice-board")

plotter = Plot(
    data_length=45,
    height_scale=60,
    pixel_shift=3,
    line_thickness=3,
    background=PlotColors.WHITE,
    height=800,
    width=1400,
    title=f"Sensor Readings",
)


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
    print(max(pressures) / 40960)
    for s in range(45):
        pressures[s] /= 4096
        pressures[s] -= 1024
    plotter.push(pressures)
