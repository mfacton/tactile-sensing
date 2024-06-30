from tools.plotter import Plot, PlotColors
from tools.serial_manager import SerialManager

manager = SerialManager("lattice-row-nine", baud=921600)

plotter = Plot(
    data_length=10,
    height_scale=30,
    pixel_shift=2,
    line_thickness=3,
    background=PlotColors.BLACK,
    height=800,
    width=1400,
    title=f"Sensor Readings",
)


def read_data():
    pressures = []
    temperatures = []
    data = manager.read_bytes(50)
    for s in range(10):
        start_index = s * 5
        pressures.append(
            data[start_index + 2] << 16 | data[start_index + 1] << 8 | data[start_index]
        )
        temperatures.append(data[start_index + 4] << 8 | data[start_index + 3])

    return pressures, temperatures


while True:
    pressures, temperatures = read_data()
    # print(max(pressures) / 40960)
    for s in range(10):
        pressures[s] /= 4096
        pressures[s] -= 1000
    plotter.push(pressures)
