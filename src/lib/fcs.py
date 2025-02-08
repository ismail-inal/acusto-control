from pipython import pitools
import cv2
import numpy as np
from lib.cmr import capture_single_image


def move_to_focus(pidevice, camera, config, coords, dz=0.005):
    """
    Function to find the sharpest image by moving the camera along the z-axis
    and calculating the sharpness based on edge detection (Canny).

    Parameters:
    - pidevice: The device object to control the stage.
    - camera: The camera object to capture images.
    - config: Configuration dictionary loaded from JSON.
    - dz: The step size for movement along the z-axis (default is 0.005).

    Returns:
    - Nothing
    """

    x0, y0, x1, y1 = coords
    sharpness_scores = []
    step_nums = np.arange(-10, 11)  # Step range from -10 to 10, inclusive

    # Get current z position
    current_z = pidevice.qPOS(config["AXES"]["z"])[config["AXES"]["z"]]

    for step_num in step_nums:
        # Move the stage along the z-axis
        target_z = current_z + dz * step_num
        pidevice.MOV(config["AXES"]["z"], target_z)
        pitools.waitontarget(pidevice, config["AXES"]["z"])

        # Capture the image at the current z position
        img = capture_single_image(camera)
        isolated_img = isolate_circle(img, x0, y0, x1, y1)

        # Apply Canny edge detection to find sharpness
        edges = cv2.Canny(isolated_img, threshold1=100, threshold2=200)
        sharpness = np.sum(edges)  # Sum of edge pixel intensities
        sharpness_scores.append(sharpness)

    # Find the index of the maximum sharpness score
    best_index = np.argmax(sharpness_scores)
    best_focus = current_z + dz * step_nums[best_index]

    pidevice.MOV(config["AXES"]["z"], best_focus)  # Move to the best focus position
    pitools.waitontarget(pidevice, config["AXES"]["z"])


def isolate_circle(image: np.ndarray, x0, y0, x1, y1):
    mask = np.zeros_like(image, dtype=np.uint8)
    cv2.rectangle(mask, (x0, y0), (x1, y1), (1, 1, 1), -1)
    isolated_image = mask * image
    return isolated_image
