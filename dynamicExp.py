from pypylon import pylon
import cv2
import numpy as np

def get_brightness(image):
    return cv2.mean(image)[0]

def adjust_camera_exposure(camera, image, target_brightness=128, tolerance=10):
    #adjust_camera_exposure(camera, image, target_brightness=100 ex.)  # Darker images
    #adjust_camera_exposure(camera, image, target_brightness=200 ex.)  # Brighter images

    current_brightness = get_brightness(image)
    exposure = camera.ExposureTime.GetValue() 

    if abs(current_brightness - target_brightness) > tolerance:
        error = target_brightness - current_brightness
        adjustment_factor = 1 + (error / 128)
        new_exposure = exposure * adjustment_factor

        new_exposure = max(min(new_exposure, camera.ExposureTime.Max), camera.ExposureTime.Min)
        camera.ExposureTime.SetValue(new_exposure)

        print(f"Adjusted Exposure: {new_exposure:.2f} µs | Brightness: {current_brightness:.2f}")

camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()
camera.ExposureAuto.SetValue("Off")  
camera.ExposureTime.SetValue(5000)   

while True:
    grab = camera.GrabOne(500)
    if grab.GrabSucceeded():
        image = grab.Array
        adjust_camera_exposure(camera, image)

        # Display image for debugging
        cv2.imshow("Live Holographic Image", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.Close()
cv2.destroyAllWindows()
