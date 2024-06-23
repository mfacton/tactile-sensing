"""Simple Line Plotter for streaming high bandwidth data"""

from enum import Enum

import cv2
import numpy as np


class PlotColors(Enum):
    """Colors for each data segment"""

    YELLOW = (255, 255, 0)
    MAGENTA = (255, 0, 255)
    RED = (255, 0, 0)
    CYAN = (0, 255, 255)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    GRAY = (128, 128, 128)
    WHITE = (255, 255, 255)
    MAROON = (128, 0, 0)
    LIME = (0, 128, 0)
    PURPLE = (128, 0, 128)
    NAVY = (0, 0, 128)
    OLIVE = (128, 128, 0)
    TEAL = (0, 128, 128)
    SILVER = (192, 192, 192)
    ORANGE = (255, 165, 0)
    BROWN = (128, 0, 0)
    DARK_CYAN = (0, 128, 128)
    BLACK = (0, 0, 0)


class Plot:
    """Line Plot class"""

    def __init__(
        self,
        width=1024,
        height=512,
        data_length=9,
        pixel_shift=1,
        height_scale=1,
        line_thickness=1,
        background=PlotColors.BLACK,
        data_colors=list(PlotColors),
        title="Plot",
    ):
        self.data_length = data_length
        self.pixel_shift = pixel_shift
        self.line_thickness = line_thickness
        self.height_scale = height_scale
        self.width = width
        self.height = height
        self.background = background
        self.canvas = np.full(
            (self.height, self.width + 1, 3), self.background.value, dtype=np.uint8
        )
        self.last_points = np.zeros((data_length,))
        self.data_colors = data_colors
        self.first_plot = True
        self.title = title

    def push(self, new_data):
        """Add next point to plot"""
        self.canvas[:, : -self.pixel_shift, :] = self.canvas[:, self.pixel_shift :, :]
        self.canvas[:, -self.pixel_shift :] = self.background.value
        for s in range(self.data_length):
            if not self.first_plot:
                cv2.line(
                    self.canvas,
                    (self.width, self.height - int(new_data[s] * self.height_scale)),
                    (
                        self.width - self.pixel_shift,
                        self.height - int(self.last_points[s]),
                    ),
                    self.data_colors[s % len(self.data_colors)].value,
                    self.line_thickness,
                )
            self.last_points[s] = new_data[s] * self.height_scale

        if self.first_plot:
            self.first_plot = False

        cv2.imshow(self.title, self.canvas)
        cv2.waitKey(1)

    def reset(self):
        """Resets the plot"""
        self.canvas = np.full(
            (self.height, self.width + 1, 3), self.background.value, dtype=np.uint8
        )
        self.canvas = np.zeros((self.height, self.width + 1, 3), dtype=np.uint8)
        self.first_plot = True

    def close(self):
        """Close plot window"""
        cv2.destroyAllWindows()
