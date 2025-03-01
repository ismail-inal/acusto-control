import numpy as np
from pipython import GCSDevice, pitools
from pypylon import pylon

from lib.cmr import save_images
from lib.cnf import Config


def capture_focus_range(
    pidevice: GCSDevice,
    camera: pylon.InstantCamera,
    config: Config,
    frame_dir: str,
) -> None:
    step_nums = np.arange(-config.movement.max_step_z, config.movement.max_step_z + 1)
    current_z = pidevice.qPOS(config.axes.z)[config.axes.z]

    for step_num in step_nums:
        target_z = current_z + config.movement.dz * step_num
        pidevice.MOV(config.axes.z, target_z)
        pitools.waitontarget(pidevice, config.axes.z)
        save_images(camera, 1, frame_dir, step_num)
