import cv2
import numpy as np
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

    # Normalize the pressure values to range 0-1
    pressure_array = np.array(pressures, dtype=np.float32)
    normalized_pressures = pressure_array - 4140000
    normalized_pressures = pressure_array / 50000.0

    # Reshape the pressure array to form a heatmap grid
    heatmap = normalized_pressures.reshape((9, 5))  # Adjust the shape as needed

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

cv2.destroyAllWindows()
