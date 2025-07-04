from os import makedirs, path

from numpy import ndarray
from pypylon import pylon


def connect_camera(
    buffer_val: int, exposure: float, frame_rate: float
) -> pylon.InstantCamera:
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    camera.MaxNumBuffer.Value = buffer_val
    camera.ExposureTime.Value = exposure
    camera.AcquisitionFrameRateEnable.Value = True
    camera.AcquisitionFrameRate.Value = frame_rate
    return camera


def return_single_image(camera: pylon.InstantCamera) -> ndarray:
    camera.StartGrabbingMax(1)

    with camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException) as result:
        if result.GrabSucceeded():
            image_array = result.GetArray()
        else:
            raise RuntimeError("Image grab failed")

    camera.StopGrabbing()

    return image_array


def save_images(
    camera: pylon.InstantCamera, num: int, file_dir: str, grab_idx: int = 0
) -> None:
    makedirs(file_dir, exist_ok=True)

    camera.StartGrabbingMax(num)

    while camera.IsGrabbing():
        with camera.RetrieveResult(2000) as result:
            if result.GrabSucceeded():
                img = pylon.PylonImage()
                img.AttachGrabResultBuffer(result)
                filename = path.join(file_dir, f"{grab_idx}.tiff")
                img.Save(pylon.ImageFileFormat_Tiff, filename)
                img.Release()
                grab_idx += 1
            else:

    camera.StopGrabbing()
