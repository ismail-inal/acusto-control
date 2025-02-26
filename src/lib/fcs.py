from typing import List

import cv2
import numpy as np
from pipython import GCSDevice, pitools
from pypylon import pylon

from lib.cmr import return_single_image
from lib.cnf import Config


def move_to_focus(
    pidevice: GCSDevice,
    camera: pylon.InstantCamera,
    config: Config,
    coords: List[int],
    dz: float = 0.005,
) -> None:
    x0, y0, x1, y1 = coords
    x0, y0 = int(x0 - 30), int(y0 - 30)
    x1, y1 = int(x1 + 30), int(y1 + 30)

    sharpness_scores = []
    step_nums = np.arange(-10, 11)

    current_z = pidevice.qPOS(config.axes.z)[config.axes.z]

    for step_num in step_nums:
        target_z = current_z + dz * step_num
        pidevice.MOV(config.axes.z, target_z)
        pitools.waitontarget(pidevice, config.axes.z)

        img = return_single_image(camera)
        isolated_img = img[y0:y1, x0:x1]

        # canny edge detection
        edges = cv2.Canny(isolated_img, threshold1=100, threshold2=200)
        sharpness = np.sum(edges)
        sharpness_scores.append(sharpness)

    best_index = np.argmax(sharpness_scores)
    best_focus = current_z + dz * step_nums[best_index]

    pidevice.MOV(config.axes.z, best_focus)
    pitools.waitontarget(pidevice, config.axes.z)
