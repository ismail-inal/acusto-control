import numpy as np
import cv2
from skimage.restoration import unwrap_phase
import tifffile


def _preproccess(
    image: np.ndarray,
    blur_kernel: tuple = (51, 51),
    threshold_blocksize: int = 11,
    morphology_kernel: tuple = (5, 5),
):
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    blur = cv2.GaussianBlur(gray, blur_kernel, cv2.BORDER_DEFAULT)

    # Adaptive thresholding
    binary = cv2.adaptiveThreshold(
        blur,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        threshold_blocksize,
        2,
    )

    # Morphological closing to enhance circle shapes
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morphology_kernel)
    morphed = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    return morphed


def _get_circle(
    image: np.ndarray,
    dp=1,
    minDist=200,
    param1=50,
    param2=25,
    minRadius=50,
    maxRadius=200,
):
    preprocessed_image = _preproccess(image)

    circles = cv2.HoughCircles(
        preprocessed_image,
        cv2.HOUGH_GRADIENT,
        dp,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minRadius,
        maxRadius=maxRadius,
    )

    return circles

def _preproccess_fft(image: np.ndarray, RAD=32):
    # Perform FFT
    fft_image = np.fft.fft2(image)

    height, width = fft_image.shape
    cy, cx = height // 2, width // 2

    # Define ROI boundaries
    x_min, x_max = 0, cx
    y_min, y_max = cy, height

    # Crop the magnitude image to the ROI
    roi = np.abs(fft_image[y_min:y_max, x_min:x_max])

    # Find the coordinates of the maximum value in the ROI
    _, _, _, max_loc = cv2.minMaxLoc(roi)
    local_max_x, local_max_y = max_loc

    # Map back to the full image coordinates
    global_max_y = y_min + local_max_y
    global_max_x = x_min + local_max_x

    # Create a mask with a circle at the detected maximum
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask, (global_max_x, global_max_y), RAD, 1, -1)  # Single channel

    # Apply the mask to the FFT
    fft_masked_image = fft_image * mask

    # Compute the shift needed to move the sideband to the center
    shift_y = -global_max_y
    shift_x = -global_max_x

    # Shift the FFT to center the sideband
    fft_centered_image = np.roll(
        np.roll(fft_masked_image, shift_y, axis=0), shift_x, axis=1
    )

    # Perform inverse FFT
    ifft_centered_image = np.fft.ifft2(fft_centered_image)

    # Calculate the phase and unwrap it
    phase_matrix = np.angle(ifft_centered_image)
    phase_matrix_unwrapped = unwrap_phase(phase_matrix)

    # Normalize the phase matrix to [0, 255] using cv2.normalize
    phase_scaled = np.zeros_like(phase_matrix_unwrapped, dtype=np.uint8)
    cv2.normalize(
        phase_matrix_unwrapped,
        phase_scaled,
        alpha=0,
        beta=255,
        norm_type=cv2.NORM_MINMAX,
        dtype=cv2.CV_8U,
    )

    return phase_scaled


    

def get_circle(
    image: np.ndarray,
    dp=1,
    minDist=200,
    param1=50,
    param2=25,
    minRadius=50,
    maxRadius=200,
    buffer=30,
):
    preprocessed_image = _preproccess(image)

    circles = cv2.HoughCircles(
        preprocessed_image,
        cv2.HOUGH_GRADIENT,
        dp,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minRadius,
        maxRadius=maxRadius,
    )

    list = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for x, y, r in circles:
            list.append(
                (x , y , r)
            )
    return list


def get_circle_fft(
    image: np.ndarray,
    dp=1,
    minDist=200,
    param1=50,
    param2=25,
    minRadius=50,
    maxRadius=200,
    buffer=30,
):
    # Preprocess the image using FFT-based method
    preprocessed_image = _preproccess_fft(image)

    # Detect circles using Hough Transform
    circles = cv2.HoughCircles(
        preprocessed_image,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minRadius,
        maxRadius=maxRadius,
    )

    # If circles are detected, round and convert to integers
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
    return circles
