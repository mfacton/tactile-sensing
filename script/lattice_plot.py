import cv2
import numpy as np

from tools.plotter import Plot, PlotColors
from tools.serial_manager import SerialManager

manager = SerialManager("lattice-board")

pout_cal_count = 20

pout_indexes = [x+30 for x in range(15)]
# pout_indexes = [14]
pout_len = len(pout_indexes)
pout_accum = [0 for _ in range(pout_len)]
pout_offset = [0 for _ in range(pout_len)]

plot_height = 512
unit_height = 5
fsr_pressure = plot_height / unit_height

plotter = Plot(
    data_length=pout_len,
    height_scale=unit_height,
    pixel_shift=4,
    line_thickness=3,
    background=PlotColors.BLACK,
    height=plot_height,
    width=1024,
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
        pout_data[pidx] /= 4096
        pout_data[pidx] += fsr_pressure / 2
    # plotter.push(pout_data)

    #map code
    heatmap = np.array(pout_data).reshape((5, 3))  # Adjust the shape as needed

    # Apply a colormap to the heatmap
    heatmap_colored = cv2.applyColorMap(
        (heatmap * 255).astype(np.uint8), cv2.COLORMAP_JET
    )

    # Resize heatmap for better visualization0
    heatmap_resized = cv2.resize(
        heatmap_colored, (500, 900), interpolation=cv2.INTER_NEAREST
    )

    # Display the heatmap
    cv2.imshow("Pressure Heatmap", heatmap_resized)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
