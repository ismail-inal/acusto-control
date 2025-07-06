from dataclasses import dataclass

import numpy as np
from pipython import GCSDevice, pitools
from pypylon import pylon

from lib.camera import return_single_image, save_images
from lib.config import Config


@dataclass
class Pos2d:
    i: int
    j: int


def auto_focus(
    pidevice: GCSDevice,
    camera: pylon.InstantCamera,
    config: Config,
):
    image_array = []
    base_z = pidevice.qPOS(config.axes.z)[config.axes.z]
    for idx in range(-config.movement.z_max_step, config.movement.z_max_step + 1, 1):
        target_z = base_z + config.movement.dz * idx
        pidevice.MOV(config.axes.z, target_z)
        pitools.waitontarget(pidevice, config.axes.z)
        img = return_single_image(camera)
        image_array.append(img)
    image_array = np.stack(image_array)

    focus_point = focus_slope_variation(
        image_array,
        Pos2d(image_array.shape[-1] // 2, image_array.shape[-2] // 2),
        15,
        21,
        Pos2d(9, 9),
    )  # 91 , 3 can sometimes work better esp. on larger images

    # -max_step to map back to idx
    target_idx = focus_point - config.movement.z_max_step
    best_target_z = base_z + config.movement.dz * target_idx
    pidevice.MOV(config.axes.z, best_target_z)
    pitools.waitontarget(pidevice, config.axes.z)


def crop_images(
    image_stack: np.ndarray, mask_size: Pos2d, center_pixel: Pos2d
) -> np.ndarray:
    c_i, c_j = mask_size.i // 2, mask_size.j // 2
    new_stack = image_stack[
        :,
        center_pixel.j - c_j : center_pixel.j + c_j + 1,
        center_pixel.i - c_i - 1 : center_pixel.i + c_i + 1,
    ]

    return new_stack


def V_s(image_stack: np.ndarray) -> np.ndarray:
    slope_sign = image_stack[:, :, 1:] > image_stack[:, :, :-1]
    return slope_sign


def P(image_stack: np.ndarray) -> np.ndarray:
    position = image_stack[:, :, 1:] ^ image_stack[:, :, :-1]
    return position


def N_s(image_stack: np.ndarray) -> np.ndarray:
    return image_stack.sum(axis=(-1, -2))


def K_one(shape: int, C_b: int) -> np.ndarray:
    mid_point = shape // 2
    x_1, x_2 = mid_point - C_b, mid_point + C_b

    mask = np.zeros(shape, dtype=int)
    mask[x_1 : x_2 + 1] = 1
    return mask


def K_two(shape: int, C_b: int) -> np.ndarray:
    mid_point = shape // 2
    x_1, x_2 = mid_point - C_b // 2, mid_point + C_b // 2

    mask = np.ones(shape, dtype=int)
    mask[x_1 : x_2 + 1] = 0
    return mask


def f(F):
    F = np.asarray(F, dtype=complex)
    M = F.size
    u = np.arange(M)[:, None]
    z = np.arange(M)[None, :]
    kernel = np.exp(1j * 2 * np.pi * u * z / M)
    abs_terms = np.abs(F[:, None] * kernel)
    f = np.sum(abs_terms, axis=0) / M
    return f


# m_prime = np.where(points == 1)[0] +1 possibly
def inflection_points(array: np.ndarray) -> np.ndarray:
    z_0, z_1, z_2 = array[:-2], array[1:-1], array[2:]
    points = ((z_0 < z_1) & (z_1 > z_2)) | ((z_0 > z_1) & (z_1 < z_2))

    m_prime = np.where(points)[0] + 1

    return m_prime


def Q(inflection_points: np.ndarray, f1: np.ndarray, f2: np.ndarray, V_b: int) -> int:
    collector = []
    for point in inflection_points:
        x1, x2 = point - V_b // 2, point + V_b // 2
        sum_window = np.abs(f2[x1 : x2 + 1]).sum()
        if sum_window == 0:
            collector.append(100000)
            continue
        fuckass_thing = f1[point] * V_b / sum_window
        collector.append(fuckass_thing)

    return inflection_points[np.argmin(collector)]


def focus_slope_variation(
    image_stack: np.ndarray,
    center_pixel: Pos2d,
    C_b: int,
    V_b: int,
    mask_size: Pos2d = Pos2d(9, 9),
):
    cropped_imgs = crop_images(image_stack, mask_size, center_pixel)

    slope_sign = V_s(cropped_imgs)

    pos = P(slope_sign)

    slope_variations = N_s(pos)

    F = np.fft.fft(slope_variations)

    K1 = K_one(F.shape[-1], C_b)
    K2 = K_two(F.shape[-1], C_b)

    f1 = f(
        F.copy() * K1,
    )
    f2 = f(F * K2)

    M_prime = inflection_points(f1.copy())

    focus_point = Q(M_prime, f1, f2, V_b)

    return focus_point


def _capture_focus_range(
    pidevice: GCSDevice,
    camera: pylon.InstantCamera,
    config: Config,
    frame_dir: str,
):
    org_z = pidevice.qPOS(config.axes.z)[config.axes.z]
    for step_num in range(
        -config.movement.z_max_step, config.movement.z_max_step + 1, 1
    ):
        target_z = org_z + config.movement.dz * step_num
        pidevice.MOV(config.axes.z, target_z)
        pitools.waitontarget(pidevice, config.axes.z)
        save_images(camera, 1, frame_dir, step_num)

    pidevice.MOV(config.axes.z, org_z)
    pitools.waitontarget(pidevice, config.axes.z)
    return
