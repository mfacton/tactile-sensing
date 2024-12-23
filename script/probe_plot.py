from tools.plotter import Plot, PlotColors
from tools.serial_manager import SerialManager

manager = SerialManager("probe-board", baud=460800)

pout_cal_count = 20

# pout_indexes = [10, 11, 12, 13, 14]
# pout_indexes = [14]
pout_indexes = [x for x in range(7)]
pout_len = len(pout_indexes)
pout_accum = [0 for _ in range(pout_len)]
pout_offset = [0 for _ in range(pout_len)]

plot_height = 1000
unit_height = 50
fsr_pressure = plot_height / unit_height

plotter = Plot(
    data_length=pout_len,
    height_scale=unit_height,
    pixel_shift=3,
    skip_frames=1,
    line_thickness=3,
    background=PlotColors.BLACK,
    height=plot_height,
    width=2000,
    title=f"Sensor Readings",
)


def read_data():
    pressures = []
    temperatures = []
    data = manager.read_bytes(40)
    for s in range(8):
        start_index = s * 5
        pressures.append(
            data[start_index + 2] << 16 | data[start_index + 1] << 8 | data[start_index]
        )
        temperatures.append(data[start_index + 4] << 8 | data[start_index + 3])

    return pressures, temperatures


def read_select():
    pressures, temperatures = read_data()
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
        pout_data[pidx] -= pout_avg
        pout_data[pidx] /= 40960
        pout_data[pidx] += fsr_pressure / 2
        # pout_data[5] = (pout_data[4] + pout_data[0])/2
    plotter.push(pout_data)
