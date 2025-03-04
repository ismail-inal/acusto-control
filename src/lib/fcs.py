from os.path import join

import cv2
import numpy as np
from pipython import GCSDevice, pitools
from pypylon import pylon

from lib.cmr import save_images
from lib.cnf import Config


def _capture_focus_range(
    pidevice: GCSDevice,
    camera: pylon.InstantCamera,
    config: Config,
    frame_dir: str,
) -> float:
    step_nums = np.arange(-config.movement.max_step_z, config.movement.max_step_z + 1)
    current_z = pidevice.qPOS(config.axes.z)[config.axes.z]

    for step_num in step_nums:
        target_z = current_z + config.movement.dz * step_num
        pidevice.MOV(config.axes.z, target_z)
        pitools.waitontarget(pidevice, config.axes.z)
        save_images(camera, 1, frame_dir, step_num)

    pidevice.MOV(config.axes.z, current_z)
    pitools.waitontarget(pidevice, config.axes.z)

    return current_z


def _fft_focus_measure(gray, low_freq_radius_ratio=0.1):
    f = np.fft.fft2(gray)
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = np.abs(fshift)

    total_energy = np.sum(magnitude_spectrum)

    rows, cols = gray.shape
    crow, ccol = rows // 2, cols // 2

    radius = int(low_freq_radius_ratio * min(rows, cols))
    Y, X = np.ogrid[:rows, :cols]
    distance = np.sqrt((Y - crow) ** 2 + (X - ccol) ** 2)
    low_freq_mask = distance <= radius

    high_freq_energy = np.sum(magnitude_spectrum[~low_freq_mask])

    ratio = high_freq_energy / total_energy
    return ratio


def move_to_focus(
    pidevice: GCSDevice,
    camera: pylon.InstantCamera,
    config: Config,
    frame_dir: str,
) -> None:
    current_z = _capture_focus_range(pidevice, camera, config, frame_dir)
    best_image_idx = 0

    best_score = -np.inf
    for idx in range(-config.movement.max_step_z, config.movement.max_step_z + 1, 1):
        path = join(frame_dir, f"{idx}.tiff")
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            print(f"Warning: Could not read image {path}. Skipping.")
            continue

        score = _fft_focus_measure(image)
        print(f"Image: {path}, FFT Focus Score: {score:.4f}")

        if score > best_score:
            best_image_idx = idx

    target_z = current_z + config.movement.dz * best_image_idx
    pidevice.MOV(config.axes.z, target_z)
    pitools.waitontarget(pidevice, config.axes.z)
    print(f"Moved to z target value: {target_z}.")
