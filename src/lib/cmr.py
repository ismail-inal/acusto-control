from pypylon import pylon
import numpy as np


def capture_single_image(camera):
    # Start grabbing a single image
    camera.StartGrabbingMax(1)
    
    while camera.IsGrabbing():
        with camera.RetrieveResult(2000) as result:
            if result.GrabSucceeded():
                # Convert the image to pylon image
                image = pylon.PylonImage()
                image.AttachGrabResultBuffer(result)
                
                # Use the ImageFormatConverter to convert the image to grayscale
                converter = pylon.ImageFormatConverter()
                converter.OutputPixelFormat = pylon.PixelType_Mono8  # Convert to grayscale (Mono8)
                converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
                
                # Convert the image
                grayscale_image = converter.Convert(image)
                
                # Convert the grayscale image to np.ndarray
                image_array = grayscale_image.Array

                image.Release()
                
                return image_array
            
    camera.StopGrabbing()



def connect_camera(buffer_val, exposure):
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    camera.MaxNumBuffer.Value = buffer_val
    camera.ExposureTime.Value = exposure
    return camera
