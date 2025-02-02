from pipython import GCSDevice, pitools
from pypylon import pylon
import cv2
import numpy as np
from lib.cmr import capture_single_image
import lib.cnf as cnf



def move_to_focus(pidevice, camera, config, (x, y, r),  dz=0.005):
    """
    Function to find the sharpest image by moving the camera along the z-axis 
    and calculating the sharpness based on edge detection (Canny).

    Parameters:
    - pidevice: The device object to control the stage.
    - camera: The camera object to capture images.
    - config: Configuration dictionary loaded from JSON.
    - dz: The step size for movement along the z-axis (default is 0.005).

    Returns:
    - best_focus: The z-axis position where the sharpest image was found.
    """
    sharpness_scores = []
    step_nums = np.arange(-5, 6)  # Step range from -10 to 10, inclusive

    # Get current z position
    current_z = pidevice.qPOS(config['AXES']["z"])[config['AXES']['z']]
    
    for step_num in step_nums:
        # Move the stage along the z-axis
        target_z = current_z + dz * step_num
        pidevice.MOV(config['AXES']["z"], target_z)
        pitools.waitontarget(pidevice, config["AXES"]['z'])

        # Capture the image at the current z position
        img = capture_single_image(camera)
        isolated_img = isolate_image(img, x, y, r)
        
        
        # Apply Canny edge detection to find sharpness
        edges = cv2.Canny(isolated_img, threshold1=100, threshold2=200)
        sharpness = np.sum(edges)  # Sum of edge pixel intensities
        sharpness_scores.append(sharpness)

    # Find the index of the maximum sharpness score
    best_index = np.argmax(sharpness_scores)
    best_focus = current_z + dz * step_nums[best_index]
    
    pidevice.MOV(config['AXES']["z"], best_focus)  # Move to the best focus position
    pitools.waitontarget(pidevice, config["AXES"]['z'])


def isolate_circle(image: np.ndarray, x, y, r):
    mask = np.zeroes_like(image, dtype=np.uint8)
    cv2.circle(mask, (x, y), int(r*1.3), (1, 1, 1), -1)
    isolated_image = mask * image
    return isolated_image



